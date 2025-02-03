# use opencv to scan cameras

import cv2

def scan_cameras():
    """Scan for cameras using OpenCV"""
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # Display camera information
            print(f"Camera {i}:")
            print(f"  Width: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            print(f"  Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
            print(f"  FPS: {cap.get(cv2.CAP_PROP_FPS)}")
            print(f"  Exposure: {cap.get(cv2.CAP_PROP_EXPOSURE)}")
            print(f"  Gain: {cap.get(cv2.CAP_PROP_GAIN)}")
            print(f"  Brightness: {cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
            print(f"  Contrast: {cap.get(cv2.CAP_PROP_CONTRAST)}")
            print(f"  Auto Exposure: {cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)}")
            cap.release()

if __name__ == "__main__":
    scan_cameras()
