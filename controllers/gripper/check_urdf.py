import ikpy.chain

# 웅철이의 URDF 파일 읽기 (joint_5 봉인 반영)
my_arm_chain = ikpy.chain.Chain.from_urdf_file(
    "robot.urdf", 
    base_elements=["link0"], 
    active_links_mask=[False, True, True, True, True, False, False]
)

print("\n🦴 --- 파이썬이 스캔한 로봇 팔 뼈대 길이 ---")
for i, link in enumerate(my_arm_chain.links):
    # translation_vector(거리) 속성이 있는 진짜 뼈대들만 출력!
    if hasattr(link, 'translation_vector'):
        print(f"[{i}] {link.name} : 다음 관절까지 거리(X,Y,Z) = {link.translation_vector}")
    else:
        print(f"[{i}] {link.name} : 기준 원점 (0, 0, 0)")