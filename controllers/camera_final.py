from controller import Robot
import numpy as np
import cv2

robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("realsense_rgb")
camera.enable(timestep)

#####emitter############

emitter = robot.getDevice("emitter")


########################


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

    # 전체 마스크 (채도/밝기 기준)
    mask_global = cv2.inRange(hsv, (0, 60, 50), (179, 255, 255))

    kernel = np.ones((5,5), np.uint8)
    mask_global = cv2.erode(mask_global, kernel, iterations=1)
    mask_global = cv2.dilate(mask_global, kernel, iterations=2)

    contours, _ = cv2.findContours(mask_global, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:

        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue

        x, y, w, h = cv2.boundingRect(c)
        if w < 20 or h < 20:
            continue

        roi_hsv = hsv[y:y+h, x:x+w]

        # 🔥 색상별 마스크 픽셀 카운트 방식
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

            mask_color = cv2.inRange(
                roi_hsv,
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

        # 최소 픽셀 조건 (노이즈 제거)
        if max_pixels < 200:
            continue

        center_x = x + w / 2
        detected_blocks.append((center_x, detected_color))

        # 화면 표시
        cv2.rectangle(bgr, (x, y), (x+w, y+h), (0,255,0), 2)
        cv2.putText(bgr, detected_color, (x, y-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        # 🔥 5초마다 왼→오 순서 출력
        if current_time - last_update_time > update_interval:

            detected_blocks.sort(key=lambda b: b[0])
            ordered_colors = [b[1] for b in detected_blocks]

            print("===== Current Order (Left → Right) =====")
            print(ordered_colors)
            print("========================================")

        #############################
            msg = ",".join(ordered_colors)   # list → string 변환
            emitter.send(msg.encode())      # string → bytes
        #############################

            last_update_time = current_time

    cv2.imshow("Blocks", bgr)
    cv2.waitKey(1)

