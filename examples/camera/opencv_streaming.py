#!/usr/bin/env python3
#
# WARNING: You should only use this approach for testing cscore on platforms that
#          it doesn't support using UsbCamera
#

import cscore as cs


import cv2
import threading
import time

camera = cs.CvSource("cvsource", cs.VideoMode.PixelFormat.kMJPEG, 1280//2, 720//2, 30)

# tell OpenCV to capture video for us
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Set the frame rate
cap.set(cv2.CAP_PROP_FPS, 60)

# Do it in another thread
def _thread():
    img = None
    frame_id = 0
    prev_time = time.time()
    while True:
        retval, img = cap.read(img)# Skip a frame
        curr_time = time.time()
        print("FPS: ", 1/(curr_time - prev_time))
        prev_time = curr_time
        frame_id += 1
        if frame_id % 2 == 0:
            small_img = cv2.resize(img, (1280//2, 720//2))
            camera.putFrame(small_img)

th = threading.Thread(target=_thread, daemon=True)
th.start()
print("Streaming Using OpenCV")


mjpegServer = cs.MjpegServer("httpserver", 8081)
mjpegServer.setSource(camera)

print("mjpg server listening at http://127.0.0.1:8081")
input("Press enter to exit...")