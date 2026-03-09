import ikpy.chain
import numpy as np
import math

urdf_file_path = "robot.urdf" 

print(f"[{urdf_file_path}] 뼈대 정보 스캔 중...")

try:
    # URDF에서 로봇 뼈대(Chain) 추출
    my_arm_chain = ikpy.chain.Chain.from_urdf_file(urdf_file_path, base_elements=["link0"])
    print("✅ 로봇 뼈대 스캔 완료! (관절 개수:", len(my_arm_chain.links), "개)")
except Exception as e:
    print("❌ URDF 파일을 읽는 중 에러가 발생했어요:", e)
    exit()

# 2. 목표 좌표 (X, Y, Z) 설정 (단위: 미터)
# 예: "로봇 밑동을 기준으로 앞으로 20cm(0.2m), 높이 15cm(0.15m)로 가라!"
target_position = [0.2, 0.0, 0.15]

# 3. 대망의 역기구학(IK) 계산!
print(f"🎯 목표 좌표: X={target_position[0]}, Y={target_position[1]}, Z={target_position[2]}")
ik_angles_rad = my_arm_chain.inverse_kinematics(target_position)

# 4. 결과 출력 (ikpy는 0번 인덱스에 항상 움직이지 않는 Base를 넣음)
print("\n✨ 계산된 모터 회전 각도 (라디안 -> 도 변환) ✨")
for i, angle_rad in enumerate(ik_angles_rad):
    # 첫 번째(0번)는 Base 링크이므로 모터가 아님
    if i == 0:
        continue
    
    # 라디안을 보기 편하게 도(Degree)로 변환
    angle_deg = math.degrees(angle_rad)
    print(f"관절 {i} (joint_{i}): {angle_deg: .2f} 도  ({angle_rad: .4f} rad)")

print("\n💡 이 각도들을 아까 우리가 만든 Webots 조종 코드에 넣으면 거기로 날아가는 거야!")