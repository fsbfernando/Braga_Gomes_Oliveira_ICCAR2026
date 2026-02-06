import numpy as np
import rtde_control
import rtde_receive
import cv2
import os
import time
import pyrealsense2 as rs

# =========================
# CONFIGURAÇÕES
# =========================
ROBOT_IP = "169.254.206.123"
rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)

pipeline = rs.pipeline()
config = rs.config()

COLOR_W, COLOR_H = 1280, 720
TARGET_W, TARGET_H = 640, 480

config.enable_stream(rs.stream.color, TARGET_W, TARGET_H, rs.format.bgr8, 30)

profile = pipeline.start(config)

for _ in range(30):
        pipeline.wait_for_frames()

print("Câmera pronta.")

def move_and_capture(pose_home, pose_april):
    
    rtde_c.moveJ(pose_home, 2.5, 1.5)
    time.sleep(0.5)
    rtde_c.moveJ(pose_april, 2.5, 1.5)
    time.sleep(0.5)

    frames = pipeline.wait_for_frames(5000)
    color_frame = frames.get_color_frame()
    if color_frame:
        img = np.asanyarray(color_frame.get_data())
    else:
        print("Erro: não foi possível capturar imagem.")
        img = None

    time.sleep(0.5)

    # pipeline.stop()


    return img


def stop_camera():
    pipeline.stop()