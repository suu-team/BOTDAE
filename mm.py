import os
import math
import threading
import queue
import time
import ikpy.chain
import numpy as np
from scipy.optimize import differential_evolution

# ---------------------------------------------------------
# 환경 변수 설정
# ---------------------------------------------------------
os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['WEBOTS_CONTROLLER_URL'] = "ipc://1234/Robotarm"

from controller import Robot

# ---------------------------------------------------------
# 1. 역기구학(IK) 설정
# ---------------------------------------------------------
print("URDF 모델 정보를 불러오는 중...")
my_arm_chain = ikpy.chain.Chain.from_urdf_file(
    "robotarm.urdf", 
    base_elements=["link0"], 
    active_links_mask=[False, True, True, True, True, True, False]
)
print("로봇 역기구학(IK) 준비 완료.")

# ---------------------------------------------------------
# 2. 관절 이름 및 초기 설정
# ---------------------------------------------------------
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
JOINT_SIGNS = {name: 1 for name in JOINT_NAMES}

INITIAL_ANGLES_DEG = {
    'joint_1': 0.0,
    'joint_2': -115.0,
    'joint_3': 120.0, 
    'joint_4': -90.0,
    'joint_5': 0.0,
    'joint_6': 0.0
}

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motors = {}
for name in JOINT_NAMES:
    motor = robot.getDevice(name)
    if motor:
        motor.setVelocity(1.0)
        motors[name] = motor

same_dir_joints = ['joint_6_mimic_L1', 'joint_6_mimic_R2', 'joint_6_mimic_R3', 'joint_6_mimic_gear', 'joint_6_mimic_R1', 'joint_6_mimic_L2', 'joint_6_mimic_L3']
same_motors = [robot.getDevice(name) for name in same_dir_joints if robot.getDevice(name)]

def set_motor_angle_rad(motor_name, target_angle_rad):
    if motor_name in motors:
        motors[motor_name].setPosition(target_angle_rad * JOINT_SIGNS.get(motor_name, 1))

# ---------------------------------------------------------
# 3. 제어 입력 터미널 스레드
# ---------------------------------------------------------
command_queue = queue.Queue()

def terminal_input_thread():
    print("\n=======================================================")
    print(" [최종 수정본] J1 회전 허용, Z=0.22, J3 양수 고정")
    print(" 1. 좌표 계산: calc X Y (Z=0.22)")
    print(" 2. 개별 이동: j관절번호 각도")
    print(" 3. 위치 확인: pos (현재 J4의 IK 계산 좌표 출력)")
    print(" 4. 초기 자세: home") 
    print("=======================================================\n")
    
    while True:
        try:
            raw_input = input("\n명령 입력: ")
            user_input = raw_input.strip()
            if not user_input: continue
            if user_input.lower() == 'exit': os._exit(0)
            
            parts = user_input.split()
            cmd = parts[0].lower()
            
            if cmd == 'calc':
                if len(parts) != 3: 
                    print("형식이 틀렸습니다. 예: calc 0.0 0.15")
                    continue
                
                try:
                    x, y = float(parts[1]), float(parts[2])
                    target_pos = np.array([x, y, 0.22]) # Z값 0.22로 고정
                except ValueError:
                    print("숫자를 입력해주세요.")
                    continue
                
                # 현재 각도 백업
                current_ik_angles = [0.0] * 7 
                for i in range(1, 6): 
                    j_name = f'joint_{i}'
                    if j_name in motors:
                        current_ik_angles[i] = motors[j_name].getTargetPosition() / JOINT_SIGNS.get(j_name, 1)

                def fitness_function(angles):
                    test_angles = [0.0, angles[0], angles[1], angles[2], current_ik_angles[4], current_ik_angles[5], 0.0]
                    fk_all = my_arm_chain.forward_kinematics(test_angles, full_kinematics=True)
                    j4_pos = fk_all[4][:3, 3]
                    error = np.linalg.norm(j4_pos - target_pos)
                    
                    # J3가 0.01보다 작으면 강한 패널티 (양수 유지)
                    if angles[2] < 0.01: 
                        error += 50.0
                    
                    # 관절이 바닥을 뚫는 것 방지
                    if fk_all[2][2, 3] < 0.03 or fk_all[3][2, 3] < 0.03:
                        error += 10.0
                        
                    return error

                active_bounds = []
                for i in range(1, 4):
                    j_name = f'joint_{i}'
                    min_pos = motors[j_name].getMinPosition()
                    max_pos = motors[j_name].getMaxPosition()
                    
                    # Webots 모터 설정이 0, 0일 경우 무제한 회전(또는 URDF) 허용
                    if min_pos == 0.0 and max_pos == 0.0:
                        urdf_bounds = my_arm_chain.links[i].bounds
                        if urdf_bounds and urdf_bounds[0] is not None:
                            low, high = urdf_bounds
                        else:
                            low, high = (-math.pi, math.pi)
                    else:
                        low, high = (min_pos, max_pos) if JOINT_SIGNS[j_name] > 0 else (max_pos, min_pos)
                    
                    # J3 양수 강제 제약
                    if j_name == 'joint_3': 
                        low = max(0.0, low) 
                    
                    active_bounds.append((low, high))

                result = differential_evolution(fitness_function, active_bounds, popsize=20, maxiter=150)
                best_angles = result.x
                ik_target = [0.0, best_angles[0], best_angles[1], best_angles[2], current_ik_angles[4], current_ik_angles[5], 0.0]
                
                print(f"\n[계산 완료] J1 각도: {math.degrees(best_angles[0]):.2f}도")
                print(f"[계산 완료] J2 각도: {math.degrees(best_angles[1]):.2f}도")
                print(f"[계산 완료] J3 각도: {math.degrees(best_angles[2]):.2f}도")
                
                for i in [1, 3, 2]:
                    command_queue.put(('move', f'joint_{i}', ik_target[i]))
                    time.sleep(1.2)

            elif cmd == 'pos':
                current_angles = [0.0] * 7
                for i in range(1, 6):
                    j_name = f'joint_{i}'
                    if j_name in motors:
                        current_angles[i] = motors[j_name].getTargetPosition() / JOINT_SIGNS.get(j_name, 1)

                fk_res = my_arm_chain.forward_kinematics(current_angles, full_kinematics=True)
                curr_j4_pos = fk_res[4][:3, 3]
                
                print(f"\n[URDF 기준 현재 J4 3D 좌표]")
                print(f" X: {curr_j4_pos[0]:.4f}")
                print(f" Y: {curr_j4_pos[1]:.4f}")
                print(f" Z: {curr_j4_pos[2]:.4f}\n")

            elif cmd.startswith('j') and len(cmd) == 2 and cmd[1].isdigit() and len(parts) == 2:
                j_num = cmd[1]
                deg = float(parts[1])
                command_queue.put(('move', f'joint_{j_num}', math.radians(deg)))
                print(f"joint_{j_num} 관절을 {deg}도로 이동합니다.")

            elif cmd in ['home', 'init']:
                for j_name in ['joint_4', 'joint_3', 'joint_2', 'joint_1']:
                    command_queue.put(('move', j_name, math.radians(INITIAL_ANGLES_DEG[j_name])))
                    time.sleep(0.5)
            else:
                print("명령 확인 필요 (calc X Y / pos / home)")

        except Exception as e:
            print(f"오류: {e}")
            
threading.Thread(target=terminal_input_thread, daemon=True).start()

# ---------------------------------------------------------
# 4. 초기 설정 및 메인 루프
# ---------------------------------------------------------
for name, deg in INITIAL_ANGLES_DEG.items():
    set_motor_angle_rad(name, math.radians(deg))

while robot.step(timestep) != -1:
    while not command_queue.empty():
        _, name, rad = command_queue.get()
        set_motor_angle_rad(name, rad)
        
    if 'joint_6' in motors:
        pos_rad = motors['joint_6'].getTargetPosition()
        for m in same_motors: m.setPosition(pos_rad)