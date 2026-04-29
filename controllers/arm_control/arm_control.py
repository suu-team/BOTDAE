from controller import Robot
import math

# 1. 초기 설정
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 각도 변환 함수 (Degree -> Radian)
def to_rad(deg):
    return deg * (math.pi / 180.0)

# 2. 장치(Motor) 가져오기
# 메인 조인트 1~6
joints = {}
for i in range(1, 7):
    name = f'joint_{i}'
    joints[name] = robot.getDevice(name)

# 모든 그리퍼 미믹(Mimic) 조인트들을 하나의 리스트로 통합
mimic_joint_names = [
    'joint_6_mimic_L1', 'joint_6_mimic_R2', 'joint_6_mimic_R3',
    'joint_6_mimic_gear', 'joint_6_mimic_R1', 'joint_6_mimic_L2', 'joint_6_mimic_L3'
]

# 장치(Motor) 객체들을 리스트에 담기
mimic_motors = []
for name in mimic_joint_names:
    motor = robot.getDevice(name)
    if motor:
        mimic_motors.append(motor)

# 3. 목표 자세 정의 (Home 및 Target)
# [joint1, joint2, joint3, joint4, joint5, joint6]
home_pose = [0, 0, 0, 0, 0, to_rad(10)]  # 원래 상태 (그리퍼 10도)
target_pose = [to_rad(45), to_rad(-10), to_rad(30), to_rad(30), to_rad(-45), to_rad(-20)] # 임의의 동작

current_target = home_pose
last_state_change = 0.0

print("Robot Arm Auto-Sequence (All Same Direction) Started...")

# 4. 메인 루프
while robot.step(timestep) != -1:
    current_time = robot.getTime()

    # 5초마다 자세 변경 (Home <-> Target)
    if current_time - last_state_change > 2:
        if current_target == home_pose:
            current_target = target_pose
            print("Moving to Target Pose...")
        else:
            current_target = home_pose
            print("Returning to Home...")
        last_state_change = current_time

    # 5. 각 조인트에 위치 명령 전달 (1번 ~ 5번)
    joints['joint_1'].setPosition(current_target[0])
    joints['joint_2'].setPosition(current_target[1])
    joints['joint_3'].setPosition(current_target[2])
    joints['joint_4'].setPosition(current_target[3])
    joints['joint_5'].setPosition(current_target[4])
    
    # 6. 그리퍼 마스터 및 모든 미믹 조인트 동기화 (전부 정방향)
    g_pos = current_target[5]
    joints['joint_6'].setPosition(g_pos)

    for motor in mimic_motors:
        motor.setPosition(g_pos) # 모든 조인트에 동일한 g_pos 적용
