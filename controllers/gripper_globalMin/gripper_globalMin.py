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

# 🌟 [추가된 부분] 유전 알고리즘(진화 연산) 도구 소환!
from scipy.optimize import differential_evolution

os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['WEBOTS_CONTROLLER_URL'] = "ipc://1234/MyRobot"

from controller import Robot

# ---------------------------------------------------------
# 1️⃣ [역기구학 세팅]
# ---------------------------------------------------------
print("🦴 URDF 뼈대 정보를 불러오는 중...")
my_arm_chain = ikpy.chain.Chain.from_urdf_file(
    "ikpy_robot.urdf", 
    base_elements=["link0"], 
    active_links_mask=[False, True, True, True, True, True, False]
)

print("✅ 로봇 내비게이션(IK) 준비 완료!")

# ---------------------------------------------------------
# 2️⃣ [오프셋 & 부호 세팅]
# ---------------------------------------------------------
JOINT_OFFSETS = {
    'joint_1': 0.048,
    'joint_2': 0.434,
    'joint_3': 1.180,
    'joint_4': 1.2,
    'joint_5': 0,
    'joint_6': math.radians(-40) 
}

# 🚀 설계도(URDF)를 완벽하게 고쳤으므로 방향 강제 보정 해제!
JOINT_SIGNS = {
    'joint_1': 1,  
    'joint_2': 1,  
    'joint_3': 1,   
    'joint_4': 1,
    'joint_5': 1,
    'joint_6': 1
}

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motors = {}
for name in JOINT_OFFSETS.keys():
    motor = robot.getDevice(name)
    if motor:
        motor.setVelocity(1.0)
        motors[name] = motor

same_dir_joints = ['joint_6_mimic_L1', 'joint_6_mimic_R2', 'joint_6_mimic_R3', 'joint_6_mimic_gear', 'joint_6_mimic_R1', 'joint_6_mimic_L2', 'joint_6_mimic_L3']
same_motors = [robot.getDevice(name) for name in same_dir_joints if robot.getDevice(name)]

def set_motor_angle_rad(motor_name, target_angle_rad):
    if motor_name in motors:
        sign = JOINT_SIGNS.get(motor_name, 1)
        corrected_target = target_angle_rad * sign
        offset_rad = JOINT_OFFSETS.get(motor_name, 0.0)
        final_rad = corrected_target + offset_rad
        motors[motor_name].setPosition(final_rad)
        return final_rad
    return None

# ---------------------------------------------------------
# 3️⃣ [수동 검증 터미널 쓰레드] 웅철이 맞춤형 디버깅 툴!
# ---------------------------------------------------------
command_queue = queue.Queue()

def terminal_input_thread():
    print("\n=======================================================")
    print(" 🛠️ IK 디버깅 모드 (계산 따로, 조종 따로!)")
    print(" 1. 계산 명령: calc X Y Z (예: calc 0.2 0.0 0.1)")
    print(" 2. 개별 관절 이동: j관절번호 각도 (예: j1 45, j2 -30)")
    print(" 3. 집게 이동: grip 각도 (예: grip 10)")
    print(" (종료하려면 exit 입력)")
    print("=======================================================\n")
    
    while True:
        try:
            # 1. 입력 받기
            raw_input = input("\n명령 입력 (calc/j1~5/grip): ")
            user_input = raw_input.strip()
            
            # 🔍 [디버깅 1] 입력된 문자열의 정체 확인
            print(f"🔍 DEBUG: 원본 문자열 -> '{user_input}' (길이: {len(user_input)})")
            
            if not user_input:
                continue
            if user_input.lower() == 'exit':
                os._exit(0)
            
            # 2. 단어 분리
            parts = user_input.split()
            print(f"🔍 DEBUG: 분리된 단어들 -> {parts} (개수: {len(parts)})")
            
            if not parts:
                continue
                
            cmd = parts[0].lower()
            
            # 💡 [모드 1] 역기구학 계산
            # 💡 [모드 1] 역기구학 계산 및 각도 출력
            if cmd == 'calc':
                if len(parts) != 4:
                    print(f"❌ 좌표가 부족합니다! 예: calc 0.2 0 0.1")
                    continue
                
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                except ValueError as ve:
                    print(f"🔥 [숫자 변환 실패] 오타가 없는지 확인해주세요: {ve}")
                    continue
                
                # 1. 🌟 [부활!] 현재 로봇 자세 읽어오기 (기준점 역할)
                current_ik_angles = [0.0] * len(my_arm_chain.links)
                for i in range(1, 6): 
                    j_name = f'joint_{i}'
                    if j_name in motors:
                        real_pos = motors[j_name].getTargetPosition()
                        sign = JOINT_SIGNS.get(j_name, 1)
                        offset = JOINT_OFFSETS.get(j_name, 0.0)
                        calc_angle = (real_pos - offset) / sign
                        low, up = my_arm_chain.links[i].bounds
                        current_ik_angles[i] = max(low, min(calc_angle, up))

                # ---------------------------------------------------------
                # 🌟 [유전 알고리즘(GA) 기반 역기구학 엔진] 🌟
                # ---------------------------------------------------------
                target_pos = np.array([x, y, z])
                
                # 1. 적합도 평가 함수 (Fitness Function): 점수가 낮을수록(오차가 적을수록) 우수한 개체!
                def fitness_function(angles):
                    # 🌟 [웅철이의 천재적 꼼수!] J5(인덱스 4)를 강제로 -J1(인덱스 0)으로 덮어쓰기!
                    modified_angles = list(angles)
                    modified_angles[4] = -modified_angles[0]
                    
                    test_angles = [0.0] + modified_angles + [0.0]
                    fk_matrix = my_arm_chain.forward_kinematics(test_angles)
                    tcp_pos = fk_matrix[:3, 3] 
                    
                    # 1. 위치 오차 (이게 유일한 최우선 목표가 됨!)
                    error = np.linalg.norm(tcp_pos - target_pos)
                    
                    # 1. 위치 오차 (XYZ 거리 맞추기)
                    error = np.linalg.norm(tcp_pos - target_pos)
                    
                    
                    # 3. 바닥 관통 패널티 (충돌 방지)
                    fk_all = my_arm_chain.forward_kinematics(test_angles, full_kinematics=True)
                    if fk_all[6][2, 3] < 0.0 or fk_all[5][2, 3] < 0.0:
                        error += 100.0 
                        
                    # 4. 전체 관절 이동 최소화 패널티 (부드러운 움직임 유지)
                    current_joints = np.array(current_ik_angles[1:6])
                    angle_diff = np.sum(np.abs(np.array(angles) - current_joints))
                    error += angle_diff * 0.01 
                        
                    return error

                # 2. 유전자(관절)의 절대 한계치(Bounds) 설정
                # URDF에 적어둔 limit 값을 읽어와서 모터가 물리적 한계를 넘지 않게 강제함
                bounds = []
                for i in range(1, 6):
                    bounds.append(my_arm_chain.links[i].bounds)

                print("🧬 유전 알고리즘 탐색 시작... (최적의 안전한 자세를 진화시키는 중!)")
                
                # 3. 진화 시작! (differential_evolution)
                # popsize=15 (15배수의 낙하산 부대 투입), maxiter=50 (50세대까지 진화)
                result = differential_evolution(
                    fitness_function, 
                    bounds, 
                    strategy='best1bin', 
                    popsize=15, 
                    maxiter=50,
                    tol=1e-3
                )
                
                # 4. 진화의 최종 승리자(가장 목표에 가깝고 바닥도 안 뚫는 각도) 추출!
                best_angles = list(result.x)
                best_angles[4] = -best_angles[0] # J5를 다시 한번 확실하게 -J1으로 묶어줌
                ik_target = [0.0] + best_angles + [0.0]
                
                print(f"🧬 진화 완료! (목표 오차: {result.fun:.5f} m)")
                

                # 💡 [디버깅] ikpy가 계산한 모든 관절의 실제 3D 좌표 뽑아보기!
                fk_all = my_arm_chain.forward_kinematics(ik_target, full_kinematics=True)
                j5_z = fk_all[5][2, 3]  # Index 5 (J5)의 Z 높이
                tcp_z = fk_all[6][2, 3] # Index 6 (TCP)의 Z 높이
                
                print("\n🕵️‍♂️ --- ikpy 뇌 속의 실제 높이(Z) 검증 ---")
                print(f"  👉 J5의 높이 : {j5_z:.4f} m (바닥에서 15cm 이상 떠있어야 정상!)")
                print(f"  👉 TCP의 높이: {tcp_z:.4f} m (빨간 공 높이 0.0149 근처면 완벽 성공!)")
                print("------------------------------------------")

                # 🚀 1. 3D 그래프 창 세팅
                fig, ax = plot_utils.init_3d_figure()

                # 🚀 2. 뼈대 그리기
                my_arm_chain.plot(ik_target, ax, target=[x, y, z])

                # 🚀 3. 축 범위 설정 (줌인: 물체 좌표 근처로 좁힘)
                #target_x = -0.262 근처
                #target_y = 0.0147 근처
                #target_z = 0.0149 근처

                zoom_range = 0.2  # 👈 중심에서 ±10cm만 보이게 세팅

                ax.set_xlim(x - zoom_range, x + zoom_range) # x: [-0.362, -0.162]
                ax.set_ylim(y - zoom_range, y + zoom_range) # y: [-0.085, 0.115]
                ax.set_zlim(z - zoom_range, z + zoom_range) # z: [-0.085, 0.115]

                # 🚀 4. 파일로 저장
                plt.savefig("ik_debug_plot_zoomed.png")
                plt.close(fig)
                print("📸 뼈대가 확대된 'ik_debug_plot_zoomed.png' 파일로 저장되었습니다!")

                print("\n📊 --- IK 수학 계산 결과 ---")
                print(f"🎯 목표 좌표: X={x}, Y={y}, Z={z}")
                for i in range(1, 6): 
                    deg = math.degrees(ik_target[i]) # 라디안 -> 도로 변환
                    print(f"  👉 joint_{i} : {deg:8.2f} 도")
                print("----------------------------")
                print("💡 확인 완료! 이제 'j1 45' 처럼 명령어를 쳐서 진짜로 닿는지 확인해보세요.")
            
                print("----------------------------")
                print("💡 계산 완료! 이제 로봇이 1번 관절부터 차례대로 자동 이동합니다 🚀")
                
                # 🌟 [여기서부터 자동화 마법 시작!] 🌟
                for i in range(1, 6):
                    j_name = f'joint_{i}'
                    
                    # ikpy가 계산해 낸 순수 정답 각도 (라디안)
                    pure_rad = ik_target[i] 
                    
                    # 1. 큐에 이동 명령 넣기 
                    # 💡 set_motor_angle_rad 함수가 오프셋을 알아서 더해주니까 순수 각도만 넘기면 돼!
                    command_queue.put(('move', j_name, pure_rad))
                    print(f"  👉 [{j_name}] 목표 각도로 이동 중...")
                    
                    # 2. 다음 관절이 움직이기 전까지 1.5초 대기 
                    time.sleep(1.5) 
                    
                print("\n✅ 모든 관절 이동 완료! TCP가 목표 좌표에 도달했습니다! 🎯")

            # 💡 [모드 2] 개별 관절 이동 (j1~j5) 
            elif cmd.startswith('j') and len(cmd) == 2 and cmd[1].isdigit() and len(parts) == 2:
                j_num = cmd[1]
                deg = float(parts[1])
                rad = math.radians(deg)
                command_queue.put(('move', f'joint_{j_num}', rad))
                print(f"🚀 joint_{j_num} 을(를) {deg}도로 이동!")
            # 💡 [모드 3] 집게 이동
            elif cmd == 'grip' and len(parts) == 2:
                grip_deg = float(parts[1])
                command_queue.put(('grip', 'joint_6', math.radians(grip_deg)))
                print(f"🤏 그리퍼 {grip_deg}도 조작!")
                
            else:
                print("❌ 명령 형식이 틀렸습니다. (예: calc 0.2 0 0.1 / j2 45)")

        except ValueError as ve:
            # 💡 단순히 "숫자 이상해"라고 하지 말고, 진짜 무슨 에러인지 찍어보자!
            print(f"🔥 진짜 에러 발생: {ve}") 
        except Exception as e:
            print(f"❌ 예상치 못한 오류: {e}")
            
threading.Thread(target=terminal_input_thread, daemon=True).start()

# ---------------------------------------------------------
# 4️⃣ [메인 루프] 
# ---------------------------------------------------------
for joint_name in JOINT_OFFSETS.keys():
    if joint_name == 'joint_6':
        set_motor_angle_rad('joint_6', math.radians(70))
    elif joint_name == 'joint_2':
        set_motor_angle_rad('joint_2', math.radians(90))
    else:
        set_motor_angle_rad(joint_name, 0.0)

while robot.step(timestep) != -1:
    while not command_queue.empty():
        cmd_type, joint_name, target_rad = command_queue.get()
        if joint_name in motors:
            set_motor_angle_rad(joint_name, target_rad)

    if 'joint_6' in motors:
        target_pos_rad = motors['joint_6'].getTargetPosition()
        for m in same_motors:
            m.setPosition(target_pos_rad)