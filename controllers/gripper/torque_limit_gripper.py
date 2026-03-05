import os
os.environ['WEBOTS_HOME'] = '/usr/local/webots'

os.environ['WEBOTS_CONTROLLER_URL'] = "ipc://1234/MyRobot"

from controller import Robot

# 1. 로봇 및 타임스텝 초기화
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# 2. 마스터 모터(joint_6) 설정
master_motor = robot.getDevice('joint_6')

# 3. 따라 움직일(Mimic) 조인트들을 그룹별로 분류
# 같은 방향으로 회전하는 조인트들
same_dir_joints = ['joint_6_mimic_L1', 'joint_6_mimic_R2', 'joint_6_mimic_R3', 'joint_6_mimic_gear', 'joint_6_mimic_R1', 'joint_6_mimic_L2', 'joint_6_mimic_L3']



# 모터 최대 토크 제한
master_motor.setAvailableTorque(4.1)


motors = {}

# 각 모터에 다른 토크 설정
motor_config = {
    "joint_1": 10.6,
    "joint_2": 10.6,
   "joint_4": 4.1,
    "joint_5": 4.1,
}

for name, torque in motor_config.items():
    m = robot.getDevice(name)
    m.setAvailableTorque(torque)
    motors[name] = m  # 저장




# 장치(Motor) 객체들을 리스트에 담기 (존재하지 않을 경우를 대비해 필터링)
same_motors = []
for name in same_dir_joints:
    motor = robot.getDevice(name)
    if motor:
        same_motors.append(motor)
        motor.setAvailableTorque(4.1)






print("Gripper sync controller started...")

# 4. 메인 루프
while robot.step(timestep) != -1:
    # joint_6의 현재 목표 위치(수동 조작 시 슬라이더 값 등)를 읽어옵니다.
    target_pos = master_motor.getTargetPosition()
    
    # 정방향 동기화 (L1, R2, R3)
    for motor in same_motors:
        motor.setPosition(target_pos)
