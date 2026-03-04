import os
import math
import threading
import queue

os.environ['WEBOTS_HOME'] = '/usr/local/webots'
os.environ['WEBOTS_CONTROLLER_URL'] = "ipc://1234/MyRobot"

from controller import Robot

# ---------------------------------------------------------
# ⚙️ [오프셋 설정]
# ---------------------------------------------------------
JOINT_OFFSETS = {
    'joint_1': -0.07,
    'joint_2': 1.355,
    'joint_3': -1.182,
    'joint_4': -1.237,
    'joint_5': 0.0,
    'joint_6': math.radians(-40)  # 그리퍼 마스터
}

robot = Robot()
timestep = int(robot.getBasicTimeStep())

motors = {}
for name in JOINT_OFFSETS.keys():
    motor = robot.getDevice(name)
    if motor:
        motors[name] = motor

        # 🚀 [추가된 마법의 코드] 모터 속도를 스무스하게 제한!
        # (단위: 라디안/초. 1.0이면 1초에 약 57도 정도 부드럽게 움직임)
        # 만약 더 천천히 움직이게 하고 싶으면 0.5로 줄여도 돼!
        motor.setVelocity(1.0)

same_dir_joints = ['joint_6_mimic_L1', 'joint_6_mimic_R2', 'joint_6_mimic_R3', 'joint_6_mimic_gear', 'joint_6_mimic_R1', 'joint_6_mimic_L2', 'joint_6_mimic_L3']
same_motors = [robot.getDevice(name) for name in same_dir_joints if robot.getDevice(name)]

def set_motor_angle_rad(motor_name, target_angle_rad):
    if motor_name in motors:
        offset_rad = JOINT_OFFSETS.get(motor_name, 0.0)
        final_rad = target_angle_rad + offset_rad
        motors[motor_name].setPosition(final_rad)
        return final_rad
    return None

# ---------------------------------------------------------
# 🎤 [입력 쓰레드] 터미널에서 실시간으로 명령을 받는 기능
# ---------------------------------------------------------
command_queue = queue.Queue()  # 메인 루프와 입력을 연결해 줄 우체통

def terminal_input_thread():
    print("\n========================================")
    print(" 🎮 실시간 관절 제어 터미널 ON")
    print(" 입력 예시: joint_1 90  (1번 관절을 90도로 이동)")
    print(" 입력 예시: joint_6 10  (그리퍼를 10도로 닫기)")
    print(" (종료하려면 exit 입력)")
    print("========================================\n")
    
    while True:
        try:
            user_input = input("명령 입력 (조인트 각도[도]): ").strip()
            if user_input.lower() == 'exit':
                print("테스트를 종료합니다.")
                os._exit(0)  # 프로그램 완전 종료
            
            parts = user_input.split()
            if len(parts) == 2:
                joint_name = parts[0]
                target_deg = float(parts[1])
                target_rad = math.radians(target_deg) # 도(Deg) -> 라디안(Rad) 자동 변환!
                
                # 우체통에 명령 넣기
                command_queue.put((joint_name, target_rad, target_deg))
            else:
                print("❌ 형식 오류! (예: joint_2 -45)")
        except ValueError:
            print("❌ 숫자만 입력해주세요! (예: joint_3 30)")
        except Exception as e:
            pass

# 메인 루프와 별개로 입력만 기다리는 쓰레드 실행
threading.Thread(target=terminal_input_thread, daemon=True).start()

# ---------------------------------------------------------
# 🚀 [초기화] Home Position
# ---------------------------------------------------------
for joint_name in JOINT_OFFSETS.keys():
    if joint_name == 'joint_6':
        set_motor_angle_rad('joint_6', math.radians(70))
    else:
        set_motor_angle_rad(joint_name, 0.0)

# ---------------------------------------------------------
# 🔄 메인 루프 (시뮬레이션 재생 유지)
# ---------------------------------------------------------
while robot.step(timestep) != -1:
    
    # 우체통(Queue)에 새로운 명령이 들어왔는지 확인하고 바로 실행!
    while not command_queue.empty():
        joint_name, target_rad, target_deg = command_queue.get()
        if joint_name in motors:
            set_motor_angle_rad(joint_name, target_rad)
            print(f"✅ [{joint_name}] {target_deg}도 ({target_rad:.2f} rad) 이동 명령 전송!")
        else:
            print(f"❌ '{joint_name}' 조인트를 찾을 수 없습니다.")

    # --- [그리퍼 동기화 로직] ---
    if 'joint_6' in motors:
        target_pos_rad = motors['joint_6'].getTargetPosition()
        for m in same_motors:
            m.setPosition(target_pos_rad)