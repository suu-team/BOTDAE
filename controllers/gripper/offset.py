import os
import math
import threading
import queue
import time
import ikpy.chain
import numpy as np

os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['WEBOTS_CONTROLLER_URL'] = "ipc://1234/MyRobot"

from controller import Robot

# ---------------------------------------------------------
# 1️⃣ [역기구학 세팅] 
# ---------------------------------------------------------
print("🦴 URDF 뼈대 정보를 불러오는 중...")
my_arm_chain = ikpy.chain.Chain.from_urdf_file(
    "robot.urdf", 
    base_elements=["link0"], 
    active_links_mask=[False, True, True, True, True, True, False]
)
print("✅ 로봇 내비게이션(IK) 준비 완료!")

# ---------------------------------------------------------
# 2️⃣ [오프셋 & 부호 세팅] Webots와 수학(IK)의 차이 보정
# ---------------------------------------------------------
JOINT_OFFSETS = {
    'joint_1': -0.07,
    'joint_2': 1.355,
    'joint_3': -1.182,
    'joint_4': -1.237,
    'joint_5': 0.0,
    'joint_6': math.radians(-40) 
}

# 🚀 Webots와 ikpy의 회전 방향이 거꾸로인 관절을 맞춰주는 스위치!
JOINT_SIGNS = {
    'joint_1': 1,
    'joint_2': -1,  # 방향이 거꾸로인 joint_2를 뒤집어 줌!
    'joint_3': -1,   # (테스트해보고 거꾸로 도는 관절이 있으면 여기를 -1로 바꿔!)
    'joint_4': -1,
    'joint_5': 1,
    'joint_6': 1
}

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motors = {}
for name in JOINT_OFFSETS.keys():
    motor = robot.getDevice(name)
    if motor:
        motor.setVelocity(1.0) # 스무스한 움직임
        motors[name] = motor

same_dir_joints = ['joint_6_mimic_L1', 'joint_6_mimic_R2', 'joint_6_mimic_R3', 'joint_6_mimic_gear', 'joint_6_mimic_R1', 'joint_6_mimic_L2', 'joint_6_mimic_L3']
same_motors = [robot.getDevice(name) for name in same_dir_joints if robot.getDevice(name)]

# 🎯 [핵심 제어 함수] 방향(Sign)과 오프셋을 동시 적용!
def set_motor_angle_rad(motor_name, target_angle_rad):
    if motor_name in motors:
        # 방향 보정
        sign = JOINT_SIGNS.get(motor_name, 1)
        corrected_target = target_angle_rad * sign
        
        # 오프셋 보정
        offset_rad = JOINT_OFFSETS.get(motor_name, 0.0)
        final_rad = corrected_target + offset_rad
        
        motors[motor_name].setPosition(final_rad)
        return final_rad
    return None

# ---------------------------------------------------------
# 3️⃣ [터미널 입력 쓰레드] 역기구학 + 충돌 방지 콤보
# ---------------------------------------------------------
command_queue = queue.Queue()

def terminal_input_thread():
    print("\n========================================")
    print(" 🎯 마스터 버전: 방향 보정 + 충돌 회피 IK 제어")
    print(" [이동] X Y Z 좌표 입력 (예: 0.2 0.0 0.15)")
    print(" [집게] grip 각도 입력 (예: grip 10)")
    print(" (종료하려면 exit 입력)")
    print("========================================\n")
    
    while True:
        try:
            user_input = input("명령 입력: ").strip()
            if user_input.lower() == 'exit':
                print("테스트를 종료합니다.")
                os._exit(0)
            
            parts = user_input.split()
            
            # 1) 좌표 이동 명령일 때 (숫자 3개)
           # 1) 좌표 이동 명령일 때 (숫자 3개)
            if len(parts) == 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                target_position = [x, y, z]

                # 현재 자세 읽어오기
                current_ik_angles = [0.0] * len(my_arm_chain.links)
                for i in range(1, 6):
                    joint_name = f'joint_{i}'
                    if joint_name in motors:
                        real_pos = motors[joint_name].getTargetPosition()
                        sign = JOINT_SIGNS.get(joint_name, 1)
                        current_ik_angles[i] = (real_pos - JOINT_OFFSETS.get(joint_name, 0.0)) * sign

                # 🚀 [핵심] 복잡한 자세 강제, 허공 띄우기 다 빼고 "순수하게 도달"만 목표로 계산!
                print(f"🎯 복잡한 명령 빼고 오직 좌표 [X:{x}, Y:{y}, Z:{z}] 만을 향해 계산 중...")
                
                ik_target = my_arm_chain.inverse_kinematics(
                    target_position=target_position,
                    initial_position=current_ik_angles
                    # 👈 원래 여기 있던 target_orientation을 완전히 지워버림!
                )

                print(" 🚀 계산 완료! 로봇팔 전개!")
                for i in range(1, 6):
                    joint_name = f'joint_{i}'
                    target_rad = ik_target[i]
                    command_queue.put(('move', joint_name, target_rad))

            elif len(parts) == 2 and parts[0].lower() == 'grip':
                grip_deg = float(parts[1])
                command_queue.put(('grip', 'joint_6', math.radians(grip_deg)))
                print(f"🤏 그리퍼를 {grip_deg}도로 조작합니다.")
                
            else:
                print("❌ 입력 형식 오류! (예: 0.2 0.0 0.15 또는 grip 10)")
                
        except ValueError:
            print("❌ 숫자만 정확히 입력해주세요!")
        except Exception as e:
            print(f"❌ 에러 발생: {e}")

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