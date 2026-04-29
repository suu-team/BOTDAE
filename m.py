import os
import math
import threading
import queue
import time
import ikpy.chain
import numpy as np
import re
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
from scipy.optimize import differential_evolution

# ---------------------------------------------------------
# 환경 변수 설정 (로봇 이름 일치)
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
# 2. 관절 이름, 부호 및 초기 각도 설정 (오프셋 제거 완료)
# ---------------------------------------------------------
JOINT_NAMES = [
    'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
]

JOINT_SIGNS = {
    'joint_1': -1,  
    'joint_2': 1,  
    'joint_3': 1,   
    'joint_4': 1,
    'joint_5': 1,
    'joint_6': 1
}

# 🌟 업데이트된 초기 자세 (0, 115, -120, 90, 0, 0)
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
    """
    목표 각도(라디안)를 입력받아 부호를 적용한 뒤 모터에 명령을 내립니다. (오프셋 제거)
    """
    if motor_name in motors:
        sign = JOINT_SIGNS.get(motor_name, 1)
        final_rad = target_angle_rad * sign
        motors[motor_name].setPosition(final_rad)
        return final_rad
    return None

# ---------------------------------------------------------
# 3. 디버깅 및 제어 입력 터미널 스레드
# ---------------------------------------------------------
command_queue = queue.Queue()

def terminal_input_thread():
    print("\n=======================================================")
    print(" IK 제어 및 디버깅 모드 (업데이트 완료 버전)")
    print(" 1. 좌표 기반 계산 및 이동: calc X Y Z (예: calc 0.2 0.0 0.1)")
    print(" 2. 개별 관절 이동: j관절번호 각도 (예: j1 45, j2 -30)")
    print(" 3. 그리퍼 이동: grip 각도 (예: grip 10)")
    print(" 4. 초기 자세로 복귀: home (또는 init)") 
    print(" (종료하려면 exit 입력)")
    print("=======================================================\n")
    
    while True:
        try:
            raw_input = input("\n명령 입력 (calc/j1~5/grip/home): ")
            user_input = raw_input.strip()
            
            if not user_input:
                continue
            if user_input.lower() == 'exit':
                os._exit(0)
            
            parts = user_input.split()
            
            if not parts:
                continue
                
            cmd = parts[0].lower()
            
            # [모드 1] 역기구학 계산 및 자동 이동
            if cmd == 'calc':
                if len(parts) != 4:
                    print(f"좌표 입력이 부족합니다. 형식: calc X Y Z")
                    continue
                
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                except ValueError as ve:
                    print(f"숫자 변환 실패. 입력값을 확인해주세요: {ve}")
                    continue
                
                target_x = x - 0.015
                target_z = z + 0.12
                print(f"입력 좌표 X={x}, Y={y}, Z={z} (보정 적용 -> 실제 타겟 X={x:.3f}m, Z={target_z:.3f}m)")
                
                current_ik_angles = [0.0] * len(my_arm_chain.links)
                for i in range(1, 6): 
                    j_name = f'joint_{i}'
                    if j_name in motors:
                        real_pos = motors[j_name].getTargetPosition()
                        sign = JOINT_SIGNS.get(j_name, 1)
                        calc_angle = real_pos / sign
                        
                        low, up = my_arm_chain.links[i].bounds
                        current_ik_angles[i] = max(low, min(calc_angle, up))

                bounds = []
                for i in range(1, 6):
                    j_name = f'joint_{i}'
                    if j_name in motors:
                        min_pos = motors[j_name].getMinPosition()
                        max_pos = motors[j_name].getMaxPosition()
                        
                        if not (min_pos == 0.0 and max_pos == 0.0):
                            sign = JOINT_SIGNS.get(j_name, 1)
                            ik_min = min_pos / sign
                            ik_max = max_pos / sign
                            
                            if sign < 0:
                                ik_min, ik_max = ik_max, ik_min
                                
                            bounds.append((ik_min, ik_max))
                            continue
                    
                    bounds.append(my_arm_chain.links[i].bounds)

                target_pos = np.array([x, y, target_z]) 
                
                def fitness_function(angles):
                    modified_angles = list(angles)
                    modified_angles[4] = current_ik_angles[5]
                    
                    test_angles = [0.0] + modified_angles + [0.0]
                    fk_all = my_arm_chain.forward_kinematics(test_angles, full_kinematics=True)
                    tcp_pos = fk_all[6][:3, 3] 
                    
                    error = np.linalg.norm(tcp_pos - target_pos) * 10.0
                    j5_axis_direction = fk_all[5][:3, 2] 
                    orientation_error = (j5_axis_direction[2] - 1.0) ** 2
                    error += orientation_error * 2.0
                    
                    if fk_all[6][2, 3] < 0.0 or fk_all[5][2, 3] < 0.0:
                        error += 100.0 
                        
                    current_joints = np.array(current_ik_angles[1:5]) 
                    angle_diff = np.sum(np.abs(np.array(modified_angles[:4]) - current_joints))
                    error += angle_diff * 0.01 
                        
                    return error

                print("유전 알고리즘을 이용한 최적 자세 탐색 시작...")
                
                result = differential_evolution(
                    fitness_function, 
                    bounds, 
                    strategy='best1bin', 
                    popsize=15, 
                    maxiter=100,
                    tol=1e-3
                )
                
                best_angles = list(result.x)
                best_angles[4] = current_ik_angles[5] 
                ik_target = [0.0] + best_angles + [0.0]
                
                print(f"탐색 완료 (목표 오차: {result.fun:.5f} m)")
                
                fk_all = my_arm_chain.forward_kinematics(ik_target, full_kinematics=True)
                j5_z = fk_all[5][2, 3]  
                tcp_z = fk_all[6][2, 1] 
                
                print("\n--- IK 모델 높이 검증 ---")
                print(f" Joint 5 높이: {j5_z:.4f} m")
                print(f" TCP (End-effector) 높이: {tcp_z:.4f} m (입력 Z {z}에서 +3cm 타겟 반영)")
                print("-------------------------")

                fig, ax = plot_utils.init_3d_figure()
                my_arm_chain.plot(ik_target, ax, target=[x, y, target_z])

                zoom_range = 0.2  
                ax.set_xlim(x - zoom_range, x + zoom_range) 
                ax.set_ylim(y - zoom_range, y + zoom_range) 
                ax.set_zlim(target_z - zoom_range, target_z + zoom_range) 

                plt.savefig("ik_debug_plot_zoomed.png")
                plt.close(fig)
                print("뼈대 줌인 이미지가 'ik_debug_plot_zoomed.png'로 저장되었습니다.")

                print("\n--- IK 계산 결과 (목표 각도) ---")
                print(f"실제 이동 타겟: X={x}, Y={y}, Z={target_z:.3f}")
                for i in range(1, 5): 
                    deg = math.degrees(ik_target[i])
                    print(f" joint_{i} : {deg:8.2f} 도")
                print("--------------------------------")
                
                print("로봇이 지정된 순서대로 이동을 시작합니다. (Joint 5, 6 고정)")
                
                move_order = [1, 4, 3, 2]
                for i in move_order:
                    j_name = f'joint_{i}'
                    pure_rad = ik_target[i] 
                    command_queue.put(('move', j_name, pure_rad))
                    print(f" [{j_name}] 목표 각도로 이동 중...")
                    time.sleep(1.5)
                    
                print("모든 관절 이동 명령이 완료되었습니다.")

            # [모드 2] 개별 관절 수동 이동
            elif cmd.startswith('j') and len(cmd) == 2 and cmd[1].isdigit() and len(parts) == 2:
                j_num = cmd[1]
                deg = float(parts[1])
                rad = math.radians(deg)
                command_queue.put(('move', f'joint_{j_num}', rad))
                print(f"joint_{j_num} 관절을 {deg}도로 이동합니다.")
                
            # [모드 3] 그리퍼 제어
            elif cmd == 'grip' and len(parts) == 2:
                grip_deg = float(parts[1])
                command_queue.put(('grip', 'joint_6', math.radians(grip_deg)))
                print(f"그리퍼를 {grip_deg}도로 조작합니다.")

            # [모드 4] 초기 자세 복귀 시퀀스
            elif cmd in ['home', 'init']:
                print("\n새로 설정된 초기 자세(Home)로 안전하게 복귀합니다...")
                
                safe_home_order = ['joint_4', 'joint_3', 'joint_2', 'joint_1', 'joint_5', 'joint_6']
                
                for j_name in safe_home_order:
                    target_deg = INITIAL_ANGLES_DEG.get(j_name, 0.0)
                    command_queue.put(('move', j_name, math.radians(target_deg)))
                    print(f" [{j_name}] 초기 각도({target_deg}도)로 복귀 중...")
                    time.sleep(0.5) 
                
                print("초기 자세 복귀 명령이 완료되었습니다.")
                
            else:
                print("명령 형식이 올바르지 않습니다. (예: calc 0.2 0 0.1 / j2 45 / home)")

        except ValueError as ve:
            print(f"값 오류 발생: {ve}") 
        except Exception as e:
            print(f"예상치 못한 오류 발생: {e}")
            
threading.Thread(target=terminal_input_thread, daemon=True).start()

# ---------------------------------------------------------
# 4. 초기 관절 설정 및 메인 루프
# ---------------------------------------------------------

# 로봇 시작 시 초기 각도 적용 
for joint_name in JOINT_NAMES:
    target_deg = INITIAL_ANGLES_DEG.get(joint_name, 0.0)
    set_motor_angle_rad(joint_name, math.radians(target_deg))

# Webots 시뮬레이션 스텝 루프
while robot.step(timestep) != -1:
    while not command_queue.empty():
        cmd_type, joint_name, target_rad = command_queue.get()
        if joint_name in motors:
            set_motor_angle_rad(joint_name, target_rad)

    # Mimic joint 동기화 처리
    if 'joint_6' in motors:
        target_pos_rad = motors['joint_6'].getTargetPosition()
        for m in same_motors:
            m.setPosition(target_pos_rad)