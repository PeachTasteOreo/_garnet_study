import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import sys
import warnings

# 경고 메시지 무시
warnings.filterwarnings('ignore')

# YOLO 모델 로드
model = YOLO('yolov8s.pt')

# 리얼센스 초기 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

# 파이프라인 시작
pipeline.start(config)
align_to = rs.stream.color
align = rs.align(align_to)

def compute_degree(x_accel, y_accel, z_accel):
    pitch = np.degrees(np.arctan(z_accel / y_accel))
    roll = np.degrees(np.arctan(x_accel / y_accel))
    yaw = np.degrees(np.arctan(z_accel / x_accel))
    return pitch, roll, yaw

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        accel_frame = frames.first_or_default(rs.stream.accel)
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if color_frame and depth_frame:
            color_image = np.asanyarray(color_frame.get_data())
            
            if accel_frame:
                raw_accel = accel_frame.as_motion_frame().get_motion_data()
                accel_data = np.array([raw_accel.x, raw_accel.y, raw_accel.z])
                pitch, roll, yaw = compute_degree(raw_accel.x, raw_accel.y, raw_accel.z)


            if gyro_frame:
                raw_gyro = gyro_frame.as_motion_frame().get_motion_data()
                gyro_data = np.array([raw_gyro.x, raw_gyro.y, raw_gyro.z])
               
            sys.stdout.write(f"\rPitch: {pitch:.3f}°, Yaw: {yaw:.3f}°, Roll: {roll:.3f}°, Accel: {accel_data}, Gyro: {gyro_data}")
            sys.stdout.flush()
            
            # YOLO 객체 탐지
            results = model.track(source=color_image, persist=True, classes=39, verbose=False)
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    pixel_x = (x1 + x2) // 2
                    pixel_y = (y1 + y2) // 2
                    depth = depth_frame.get_distance(pixel_x, pixel_y) if depth_frame else 0.0
                    label = f"{model.names[int(box.cls[0])]}, {depth:.3f}m"
                    cv2.rectangle(color_image, (x1, y1), (x2, y2), (64, 224, 208), 1, cv2.LINE_AA)
                    cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (127, 255, 212), 1)
            
            cv2.imshow("Color Image", color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\n프로그램 종료")
