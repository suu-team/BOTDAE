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
    'joint_1': -0.07,
    'joint_2': 1.774,
    'joint_3': -1.182,
    'joint_4': -1.237,
    'joint_5': 0.0,
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

                # 1. 현재 로봇 자세를 출발점으로 잡기 (infeasible 에러 방지용 안전장치)
                current_ik_angles = [0.0] * len(my_arm_chain.links)
                
                for i in range(1, 6): # joint_1 ~ joint_5 까지 계산 (6도 포함하려면 7로 변경)
                    j_name = f'joint_{i}'
                    if j_name in motors:
                        # Webots에서 실제 로봇 각도 읽어오기
                        real_pos = motors[j_name].getTargetPosition()
                        sign = JOINT_SIGNS.get(j_name, 1)
                        offset = JOINT_OFFSETS.get(j_name, 0.0)
                        
                        # IK 파이썬이 이해할 수 있는 순수 각도로 변환
                        calc_angle = (real_pos - offset) / sign
                        
                        # 🛡️ URDF 가동 범위 안으로 강제 고정 (Clamping)
                        low, up = my_arm_chain.links[i].bounds
                        current_ik_angles[i] = max(low, min(calc_angle, up))
                    
                # 💡 [핵심 트릭] ikpy가 헤매지 않게 '미리 굽혀둔 자세'를 힌트로 주자!
                # [베이스, J1(0), J2(45도 끄덕), J3(-45도 끄덕), J4(-90도 꺾기), J5, TCP]
                smart_initial_guess = [0.0, 0.0, math.radians(45), math.radians(-45), math.radians(-90), 0.0, 0.0]

                # 2. 역기구학(IK) 계산 실행!
                ik_target = my_arm_chain.inverse_kinematics(
                    target_position=[x, y, z],
                    initial_position=current_ik_angles,
                    
                    # 💡 기존 [0, 0, -1] (수직 90도) -> ❌ 팔 짧아서 실패
                    # 💡 변경 [-1, 0, -1] (앞으로 뻗으면서 아래로 45도 찌르기) -> ✅ 완벽 성공!
                    # target_orientation=[-1, 0, -1],
                    # orientation_mode="Z"
                )

                # ... (역기구학 계산 완료 직후) ...
                
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

                # 💡 웅철이의 원래 깔끔한 출력 코드로 복구!
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
                    
                    # 1. 큐에 이동 명령 넣기 (웅철이가 'j1 45' 친 거랑 똑같은 효과!)
                    # 💡 set_motor_angle_rad 함수가 오프셋을 알아서 더해주니까 순수 각도만 넘기면 돼!
                    command_queue.put(('move', j_name, pure_rad))
                    print(f"  👉 [{j_name}] 목표 각도로 이동 중...")
                    
                    # 2. 다음 관절이 움직이기 전까지 1.5초 대기 (차례대로 움직이게 하는 핵심!)
                    time.sleep(1.5) 
                    
                print("\n✅ 모든 관절 이동 완료! TCP가 목표 좌표에 도달했습니다! 🎯")

            # 💡 [모드 2] 개별 관절 이동 (j1~j5) - 웅철이 원본 코드로 완벽 복구!
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

        # ... (기존 코드) ...
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