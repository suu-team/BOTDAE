from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("realsense_rgb")
camera.enable(timestep)

##### emitter #####
emitter = robot.getDevice("emitter")
emitter.setChannel(1)
###################

# [사용자 기준] 기존 변수들
min_area = 1500
max_area = 10000
last_update_time = 0
update_interval = 5000  # 5초

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

    # ==========================================
    # [🔥 핵심 수정 1] - 강력한 명암(밝기) 기반 마스크 생성
    # 기존 코드의 mask_global을 수정합니다.
    # HSV에서 V(Value)의 최솟값을 대폭 높여(예: 150 이상) 어두운 영역을 확실히 제거합니다.
    # S(Saturation) 값도 적절히 높여(예: 100 이상) 배경의 저채도 노이즈를 날립니다.
    # 사용자의 조명 환경에 따라 100, 150 값을 조금씩 조정하세요.
    mask_brightness = cv2.inRange(hsv, (0, 100, 150), (179, 255, 255))
    # ==========================================

    kernel = np.ones((5,5), np.uint8)
    # 기존 erosion/dilation을 닫힘(CLOSE)과 열림(OPEN) 연산으로 최적화하여 
    # 밝기 마스크의 미세한 구멍을 메우고 노이즈를 제거합니다.
    mask_clean = cv2.morphologyEx(mask_brightness, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel, iterations=1)

    # [🔥 핵심 수정 2] - '정제된 밝기 마스크'에서 컨투어를 찾습니다.
    # 이 컨투어의 바운딩 박스는 이제 블록의 밝은 앞면만 감싸게 됩니다.
    contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:

        area = cv2.contourArea(c)
        # 면적 필터링 [사용자 기준 유지]
        if area < min_area or area > max_area:
            continue

        # 이제 이 바운딩 박스(x, y, w, h)는 '꺾여서 어두운 부분'을 제외한
        # 밝은 정면만 딱 맞게 감싼 좌표입니다.
        x, y, w, h = cv2.boundingRect(c)
        if w < 20 or h < 20:
            continue

        # [🔥 핵심 수정 3] - 정제된 바운딩 박스를 사용해 새로운 ROI를 추출합니다.
        # 이 새로운 ROI는 어두운 픽셀을 포함하지 않습니다.
        roi_hsv_clean = hsv[y:y+h, x:x+w]

        # 색상 범위 [사용자 기준 유지]
        color_ranges = {
            "red1":  ((0, 100, 80), (10, 255, 255)),
            "red2":  ((170, 100, 80), (179, 255, 255)),
            "orange":((10, 100, 80), (20, 255, 255)),
            "yellow":((20, 100, 80), (35, 255, 255)),
            "green": ((35, 80, 80), (85, 255, 255)),
            "blue":  ((90, 100, 80), (120, 255, 255)),
            "navy":  ((120, 100, 60), (140, 255, 255)),
            "purple":((140, 80, 80), (170, 255, 255))
        }

        max_pixels = 0
        detected_color = "unknown"

        for color, (lower, upper) in color_ranges.items():

            # [🔥 핵심 수정 4] - 어두운 부분이 배제된 정제된 ROI에서 색상 투표를 합니다.
            mask_color = cv2.inRange(
                roi_hsv_clean,
                np.array(lower),
                np.array(upper)
            )

            pixels = cv2.countNonZero(mask_color)

            if pixels > max_pixels:
                max_pixels = pixels
                detected_color = color

        # red1 + red2 통합
        if detected_color in ["red1", "red2"]:
            detected_color = "red"

        # 노이즈 제거 [사용자 기준 유지]
        if max_pixels < 200:
            continue

        center_x = x + w / 2
        detected_blocks.append((center_x, detected_color))

        # 화면 표시 [사용자 기준 유지 - 이제 박스가 앞면만 감쌀 것입니다]
        cv2.rectangle(bgr, (x, y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(bgr, detected_color, (x, y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

    # 5초마다 왼→오 순서로 출력 [사용자 기준 유지]
    if current_time - last_update_time > update_interval:

        detected_blocks.sort(key=lambda b: b[0])
        ordered_colors = [b[1] for b in detected_blocks]

        print("emitter")
        print(ordered_colors)

        msg = ",".join(ordered_colors)
        emitter.send(msg.encode())

        last_update_time = current_time

    cv2.imshow("Blocks", bgr)
    # [🔥 핵심 수정 5] - 밝기 마스크의 정제 과정을 확인하려면 아래 주석을 푸세요
    # cv2.imshow("Clean Brightness Mask", mask_clean)
    cv2.waitKey(1)