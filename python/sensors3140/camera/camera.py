
import cscore as cs
import cv2
import threading
import time
import numpy as np
import logging
import json
import os

_logger = logging.getLogger(__name__)



class Camera:
    def __init__(self, camera_id=None, frame_size=None, fps=None, **kwargs):
        try:
            self.camera_id = int(camera_id)
        except:
            self.camera_id = camera_id

        self.frame_size = frame_size
        self.width, self.height = frame_size

        self.cap = cv2.VideoCapture(self.camera_id)

        # Required to set up the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.frame_id = 0
        self.prev_time = time.time()
        self.last_frame = None

        # Optional camera settings
        if 'exposure' in kwargs:
            self.cap.set(cv2.CAP_PROP_EXPOSURE, kwargs['exposure'])
        if 'gain' in kwargs:
            self.cap.set(cv2.CAP_PROP_GAIN, kwargs['gain'])

        self.camera_matrix = None
        if 'matrix' in kwargs:
            self.camera_matrix = np.array(kwargs['matrix'], dtype=np.float32).reshape((3, 3))
        
        self.dist_coeffs = None
        if 'distortion' in kwargs:
            self.dist_coeffs = np.array(kwargs['distortion'], dtype=np.float32)

        # Start the capture thread
        self.start_capture()

    def start_capture(self):
        threading.Thread(target=self._capture_thread, daemon=True).start()

    def setCameraCalibration(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def isCalibrated(self):
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            # Check if the camera matrix and distortion coefficients are valid
            if self.camera_matrix.shape == (3, 3) and self.dist_coeffs.shape[0] == 5:
                return True
        return False # Default to false

    def _capture_thread(self):
        img = None
        while True:
            retval, img = self.cap.read(img)
            curr_time = time.time()
            calibrated = self.isCalibrated()
            print(f"CameraID: {self.camera_id} {self.width} {self.height} FPS: {1/(curr_time - self.prev_time):.2f} Calibrated: {calibrated}")
            self.prev_time = curr_time
            self.frame_id += 1
            self.last_frame = img

    def get_frame(self):
        return self.last_frame, self.frame_id, self.prev_time
    

def load_cameras_from_config_directory(config_dir: str = None) -> dict:
    """Load camera configurations from a directory"""
    if config_dir is None:
        # default directory is ~/sensors3140/
        home_dir = os.path.expanduser("~")
        config_dir = os.path.join(home_dir, 'sensors3140')

    cameras = {}
    for file in os.listdir(config_dir):
        if file.startswith('camera') and file.endswith('.json'):
            print(f"Loading camera configuration from {file}")
            with open(os.path.join(config_dir, file)) as f:
                config = json.load(f)
                print(config)
                camera = create_camera_from_config(config)
                cameras[camera.camera_name] = camera

    return cameras

def create_camera_from_config(config: dict) -> Camera:
    """Create a Camera instance from a configuration dictionary"""
    camera_name = config.get('name', 'unknown')
    device_id = config.get('camera_id', 0)
    width,height = config.get('frame_size', (640, 480))
    fps = config.get('fps', 30)

    print(f"Creating camera {camera_name} with device ID {device_id}")
    print(f"    Resolution: {width}x{height} @ {fps} FPS")
    
    camera = Camera(
        camera_name=camera_name,
        camera_id=device_id,
        width=width,
        height=height,
        fps=fps
    )

    try:
        # load the camera calibration
        camera_matrix = np.array(config['camera_matrix'], dtype=np.float).reshape((3, 3))
        distortion = np.array(config['distortion'], dtype=np.float32)
        camera.setCameraCalibration(camera_matrix, distortion)
    except:
        _logger.warning(f"Could not load camera calibration for {device_id}")

    _logger.info(f"Created camera {camera_name} with device ID {device_id}")

    return camera


if __name__ == "__main__":
    cameras = load_cameras_from_config_directory()
    while True:
        for camera in cameras.values():
            #print(camera.camera_name)
            frame = camera.get_frame()
            if frame is not None:
                cv2.imshow(f"camera_{camera.camera_id}", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()