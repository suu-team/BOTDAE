from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("realsense_rgb")
camera.enable(timestep)

# Depth
depth_sensor = robot.getDevice("realsense_depth") 
depth_sensor.enable(timestep)

##### emitter #####
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
###################

min_area = 500
max_area = 5000

last_update_time = 0
update_interval = 5000  # 5초

fov = depth_sensor.getFov()  # 시야각(Radian)
width = depth_sensor.getWidth()   # 640
height = depth_sensor.getHeight() # 480

# 초점 거리(Focal Length) 계산!
fx = width / (2.0 * np.tan(fov / 2.0))
fy = fx  # Webots는 픽셀이 정사각형이라 fx, fy가 같아!
cx = width / 2.0  # 320
cy = height / 2.0 # 240

# 값 튀는 현상(노이즈)을 잡기 위한 메모리장치!
block_history = {"red": [], "yellow": [], "orange": []}
history_length = 30  # 최근 30프레임 평균

last_update_time = 0

while robot.step(timestep) != -1:

    current_time = robot.getTime() * 1000
    detected_blocks = []

    # Webots 이미지 → numpy
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    img = np.frombuffer(image, np.uint8).reshape((height, width, 4))
    bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # 뎁스 이미지 가져오기 
    depth_image = depth_sensor.getRangeImageArray()

    # 1. 색상 범위 세팅 
    color_ranges = {
        "red": [
            ((0, 50, 50), (10, 255, 255)),    # red1 범위
            ((160, 50, 50), (179, 255, 255))  # red2 범위
        ],
        "orange": [ ((10, 100, 80), (20, 255, 255)) ],
        "yellow": [ ((20, 100, 80), (35, 255, 255)) ]
    }

    kernel = np.ones((5,5), np.uint8)

    # 2. 색깔을 하나씩 꺼내면서 검사 시작!
    for color_name, ranges in color_ranges.items():
        
        # 🌟 핵심: 빈 까만색 도화지를 하나 준비함
        mask_color = np.zeros((height, width), dtype=np.uint8)
        
        # 해당 색깔에 정의된 모든 범위(빨간색은 2개, 나머진 1개)를 도화지에 덧칠함
        for (lower, upper) in ranges:
            mask_temp = cv2.inRange(hsv, np.array(lower), np.array(upper))
            mask_color = cv2.bitwise_or(mask_color, mask_temp) 
        
        # 노이즈 제거 (침식 1번, 팽창 1번으로 깔끔하게)
        mask_color = cv2.erode(mask_color, kernel, iterations=1)
        mask_color = cv2.dilate(mask_color, kernel, iterations=1)

        # 해당 색깔의 윤곽선(덩어리) 찾기
        contours, _ = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            # 면적으로 노이즈 및 너무 큰 쓰레기 걸러내기
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue

            x, y, w, h = cv2.boundingRect(c)
            if w < 20 or h < 20:
                continue

            # red1, red2는 'red'로 이름 통일
            final_color = "red" if color_name in ["red1", "red2"] else color_name

            # 중심점 계산
            center_x = int(x + w / 2)
            center_y = int(y + h / 2)

            # 뎁스 배열 인덱스 초과 방지 (안전벨트)
            max_y = len(depth_image) - 1
            max_x = len(depth_image[0]) - 1
            
            safe_x = max(0, min(center_x, max_x))
            safe_y = max(0, min(center_y, max_y))

            # 🌟 [y][x] 로 거리 가져오기!
            surface_Z = depth_image[safe_y][safe_x]
            
            real_X = (center_x - cx) * surface_Z / fx
            real_Y = (center_y - cy) * surface_Z / fy
            real_Z = surface_Z + 0.015

            cam_world_x, cam_world_y, cam_world_z = -0.505, -0.0913, 0.03

            # 카메라 기준 상대 좌표를 맵 기준 절대 좌표로 변환
            world_X = cam_world_x + real_Z  # 카메라가 앞(+X)을 보고 있으므로 Z를 X에 더함
            world_Y = cam_world_y - real_X  # 카메라의 오른쪽(+X)은 맵의 왼쪽(-Y)
            world_Z = cam_world_z - real_Y  # 카메라의 아래쪽(+Y)은 맵의 아래쪽(-Z)


            # 이제 리스트에 2D 픽셀 대신, 진짜 3D 좌표를 쑤셔 넣기!
            if final_color in block_history:
                block_history[final_color].append((world_X, world_Y, world_Z))
                
                # 메모장에 데이터가 15개가 넘어가면? 제일 오래된 과거 데이터 1개 지우기!
                if len(block_history[final_color]) > history_length:
                    block_history[final_color].pop(0)

            # 화면에 박스 칠 때 진짜 좌표(X, Z)를 띄워보자!
            cv2.rectangle(bgr, (x, y), (x+w, y+h), (0,255,0), 2)
            cv2.putText(bgr, f"{final_color} (X:{real_X:.2f}, Z:{real_Z:.2f})", (x, y-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)


    # 🔥 5초마다 무전기 송신부
    if current_time - last_update_time > update_interval:
        
        msg_list = []
        
        # 고정된 순서로 하나씩 꺼내기!
        for color in ["red", "yellow", "orange"]:
            
            # 메모장에 모아둔 데이터가 있다면? 평균 계산!
            if len(block_history[color]) > 0:
                # X, Y, Z 각각 모아둔 숫자들의 합 / 개수 = 평균(Average)
                avg_x = sum([pos[0] for pos in block_history[color]]) / len(block_history[color])
                avg_y = sum([pos[1] for pos in block_history[color]]) / len(block_history[color])
                avg_z = sum([pos[2] for pos in block_history[color]]) / len(block_history[color])
                
                msg_list.append(f"{color}:{avg_x:.3f}:{avg_y:.3f}:{avg_z:.3f}")
                
                # 무전 보냈으면 메모장 싹 비우기 
                block_history[color].clear() 

        msg = ",".join(msg_list)

        print("📡 emitter 전송:", msg)
        emitter.send(msg.encode())

        last_update_time = current_time
    cv2.imshow("Blocks", bgr)
    depth_array_cv = np.array(depth_image, dtype=np.float32)
    max_range = depth_sensor.getMaxRange() 

    depth_visual = (depth_array_cv / max_range * 255).astype(np.uint8)
    
    cv2.imshow("Depth Camera View", depth_visual)
    cv2.waitKey(1)
