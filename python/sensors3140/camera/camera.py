
import cscore as cs
import cv2
import threading
import time
import numpy as np
import logging
import json
import os
from sensors3140.apriltag.detector import AprilTagDetector

_logger = logging.getLogger(__name__)

class FrameData:
    def __init__(self, frame, frame_id, timestamp):
        self.frame = frame
        self.frame_id = frame_id
        self.timestamp = timestamp

    def __repr__(self):
        return f"FrameData(frame_id={self.frame_id}, timestamp={self.timestamp})"

class Camera:
    def __init__(self, camera_id=None, frame_size=None, fps=None, frame_stas=False, **kwargs):
        try:
            self.camera_id = int(camera_id)
        except:
            self.camera_id = camera_id

        self.frame_stats_enabled = frame_stas

        self.fps = fps
        self.frame_size = frame_size
        self.width, self.height = frame_size

        self.cap = cv2.VideoCapture(self.camera_id)

        # Required to set up the camera
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.current_fps = -1.0
        self.ave_fps = -1.0

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

        if 'parameters' in kwargs:
            self.parameters = np.array(kwargs['parameters'], dtype=np.float32)

        self.apriltag_detector = None

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
            frame_data = FrameData(img, self.frame_id, curr_time)
            calibrated = self.isCalibrated()
            self.current_fps = 1.0 / (curr_time - self.prev_time)  # Calculate FPS
            if self.ave_fps < 0.0:
                self.ave_fps = self.current_fps
            else:
                mix_ratio = min(1.0/self.fps,0.1)
                self.ave_fps = (1-mix_ratio) * self.ave_fps + mix_ratio * self.current_fps
            self.prev_time = curr_time
            self.frame_id += 1
            self.last_frame = img

            # Compute current frame data: min, max, mean, std
            frame_stats = []
            if self.frame_stats_enabled:
                frame_stats = [np.min(img), np.max(img), np.mean(img), np.std(img)]

                #denoised image from a median filter
                denoised_img = cv2.medianBlur(img, 5)
                noise_img = img - denoised_img
                noise_est = np.std(noise_img)
                frame_stats.append(noise_est)

                # compute an dirivative score
                sobelx = cv2.Sobel(denoised_img, cv2.CV_64F, 1, 0, ksize=5)
                sobely = cv2.Sobel(denoised_img, cv2.CV_64F, 0, 1, ksize=5)
                sobel = np.sqrt(sobelx**2 + sobely**2)
                sobel_score = np.mean(sobel)
                frame_stats.append(sobel_score)

                self.frame_stats = frame_stats

                
            if False and self.apriltag_detector is not None:
                if calibrated:
                    detections = self.apriltag_detector(img)
                    frame_data.detections = detections
                    #for detection in detections:
                    #    print(f"Camera {self.camera_id} detected tag {detection['id']}")
                    #    print(f"    Distance: {detection['distance']:.2f}m, Bearing: {detection['bearing']:.2f}°, Azimuth: {detection['azimuth']:.2f}°")
                    #    print(f"    Pose Decomposed: {detection['pose_decomposed']}")
                    
                else:
                    print(f"Camera {self.camera_id} is not calibrated")

    def get_frame(self):
        frame_data = FrameData(self.last_frame, self.frame_id, self.prev_time)
        return frame_data
    
    def enable_apriltags(self):
        self.apriltag_detector = AprilTagDetector(camera_params=self.parameters)
        pass

    def get_fov(self):
        '''Estimate the x and y fov in degrees from the camera size and parameters.'''
        fx,fy,cx,cy = self.parameters
        fov_x = 2 * np.arctan(self.width / (2 * fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(self.height / (2 * fy)) * 180 / np.pi
        return fov_x, fov_y
    
    def get_exposure(self):
        return self.cap.get(cv2.CAP_PROP_EXPOSURE)
    
    def get_gain(self):
        return self.cap.get(cv2.CAP_PROP_GAIN)
    
    def get_frame_stats(self):
        try:
            if self.frame_stats_enabled:
                return self.frame_stats
            else:
                return [-1,-1,-1,-1,-1,-1]
        except:
            return [-1,-1,-1,-1,-1,-1]
    
        

    

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