import cscore as cs
import cv2
import threading
import time
import numpy as np
import logging
import json
import os
from sensors3140.apriltags.detector import AprilTagDetector
from sensors3140.tables.network_tables import NetworkTablesManager

_logger = logging.getLogger(__name__)

class FrameData:
    """
    Container for frame data with metadata
    
    Attributes:
        frame: The image data as a numpy array
        frame_id: Unique identifier for the frame
        timestamp: When the frame was captured (time.time())
    """
    def __init__(self, frame, frame_id, timestamp):
        self.frame = frame
        self.frame_id = frame_id
        self.timestamp = timestamp
        # Optional field for detections
        self.detections = None

    def __repr__(self):
        return f"FrameData(frame_id={self.frame_id}, timestamp={self.timestamp})"

class Camera:
    """
    Camera abstraction that handles frame capture in a separate thread
    
    Provides access to camera frames with consistent timing and 
    optional frame statistics for image quality analysis
    """
    def __init__(self, camera_id=None, width=640, height=480, fps=30, 
                 enable_stats=False, name=None, **kwargs):
        """
        Initialize a camera with the specified parameters
        
        Args:
            camera_id: Integer or string identifying the camera
            width: Frame width in pixels
            height: Frame height in pixels
            fps: Frames per second
            enable_stats: Whether to compute frame quality statistics
            name: Human-readable name for this camera
            **kwargs: Additional camera parameters (exposure, gain, etc.)
        """
        self.name = name if name is not None else f"Camera_{camera_id}"
        
        # Convert camera_id to int if possible, otherwise keep as string (for file paths)
        try:
            self.camera_id = int(camera_id)
        except (ValueError, TypeError):
            self.camera_id = camera_id
        
        if self.camera_id is None:
            _logger.error(f"Invalid camera ID: {camera_id}")
            raise ValueError(f"Invalid camera ID: {camera_id}")

        self.stats_enabled = enable_stats
        self.fps_target = fps
        self.width = width
        self.height = height
        
        # FPS tracking
        self.current_fps = 0.0
        self.average_fps = 0.0
        self.frame_id = 0
        self.prev_frame_time = time.time()
        self.current_frame = None
        self.frame_stats = [-1, -1, -1, -1, -1, -1]
        
        # For periodic logging
        self.last_log_time = time.time()
        self.frames_since_log = 0
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_params = None
        
        # Initialize camera parameters from kwargs
        if 'matrix' in kwargs:
            self.camera_matrix = np.array(kwargs['matrix'], dtype=np.float32).reshape((3, 3))
        if 'distortion' in kwargs:
            self.dist_coeffs = np.array(kwargs['distortion'], dtype=np.float32)
        if 'parameters' in kwargs:
            self.camera_params = np.array(kwargs['parameters'], dtype=np.float32)
            
        # Initialize AprilTag detector
        self.apriltag_detector = None
        
        # Connect to the camera
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self.camera_id}")
            
            # Configure camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps_target)
            
            # Check if camera settings were applied correctly
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            if abs(actual_width - self.width) > 1 or abs(actual_height - self.height) > 1:
                _logger.warning(f"Camera {self.name} resolution set to {actual_width}x{actual_height} "
                               f"(requested {self.width}x{self.height})")
                self.width = int(actual_width)
                self.height = int(actual_height)
            
            # Apply optional camera settings
            if 'exposure' in kwargs:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, kwargs['exposure'])
            if 'gain' in kwargs:
                self.cap.set(cv2.CAP_PROP_GAIN, kwargs['gain'])
                
            _logger.info(f"Initialized {self.name} (ID: {self.camera_id}) with resolution "
                        f"{self.width}x{self.height} @ {self.fps_target} FPS")
            
            # Start the capture thread
            self._start_capture()
            
        except Exception as e:
            _logger.error(f"Failed to initialize camera {self.camera_id}: {str(e)}")
            raise

    def _start_capture(self):
        """Start background thread for continuous frame capture"""
        threading.Thread(target=self._capture_thread, daemon=True).start()

    def set_camera_calibration(self, camera_matrix, dist_coeffs):
        """Set camera calibration parameters for undistortion"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def is_calibrated(self):
        """Check if camera has valid calibration parameters"""
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            # Check if the camera matrix and distortion coefficients are valid
            if self.camera_matrix.shape == (3, 3) and len(self.dist_coeffs) >= 4:
                return True
        return False

    def get_camera_params(self):
        """Get the camera intrinsic parameters"""
        return self.camera_params

    def get_dist_coeffs(self):
        """Get the camera distortion coefficients"""
        return self.dist_coeffs

    def _capture_thread(self):
        """
        Background thread that continuously captures frames
        
        Updates FPS metrics and optionally computes image quality statistics
        """
        img = None
        connection_errors = 0
        max_errors = 5  # Maximum consecutive errors before logging a warning
        
        while True:
            try:
                retval, img = self.cap.read()
                curr_time = time.time()
                
                if not retval:
                    connection_errors += 1
                    if connection_errors >= max_errors:
                        _logger.warning(f"Failed to capture frames from {self.name} "
                                       f"(ID: {self.camera_id}) - {connection_errors} consecutive errors")
                        connection_errors = 0
                    time.sleep(0.1)  # Wait before trying again
                    continue
                
                # Reset error counter on successful frame capture
                connection_errors = 0
                    
                # Update FPS calculations
                frame_time_diff = curr_time - self.prev_frame_time
                if frame_time_diff > 0:
                    self.current_fps = 1.0 / frame_time_diff
                    
                    # Initialize or update moving average
                    if self.average_fps <= 0.0:
                        self.average_fps = self.current_fps
                    else:
                        # Weighted moving average for smoother FPS readings
                        mix_ratio = min(1.0/self.fps_target, 0.1)
                        self.average_fps = (1-mix_ratio) * self.average_fps + mix_ratio * self.current_fps
                
                self.prev_frame_time = curr_time
                self.frame_id += 1
                self.current_frame = img.copy()  # Create a copy to avoid race conditions
                self.frames_since_log += 1
                
                # Periodic logging - every 10 seconds
                if curr_time - self.last_log_time >= 10:
                    frame_size_mb = (self.width * self.height * 3) / (1024 * 1024)  # Size in MB for RGB image
                    _logger.info(f"{self.name}: Captured {self.frames_since_log} frames in 10s, "
                                f"size: {self.width}x{self.height} ({frame_size_mb:.2f}MB), "
                                f"average FPS: {self.average_fps:.2f}")
                    self.last_log_time = curr_time
                    self.frames_since_log = 0
    
                # Compute image quality statistics if enabled
                if self.stats_enabled:
                    self._compute_frame_stats(img)
    
            except Exception as e:
                _logger.error(f"Error in capture thread for {self.name}: {str(e)}")
                time.sleep(1.0)  # Wait before trying again
                
    def _compute_frame_stats(self, img):
        """Compute image quality statistics for a frame"""
        try:
            # Basic image statistics
            stats = [np.min(img), np.max(img), np.mean(img), np.std(img)]
    
            # Noise estimation using median filter difference
            denoised_img = cv2.medianBlur(img, 5)
            noise_img = img - denoised_img
            noise_est = np.std(noise_img)
            stats.append(noise_est)
    
            # Edge content estimation using Sobel operator
            gray_img = cv2.cvtColor(denoised_img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else denoised_img
            sobelx = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0, ksize=5)
            sobely = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1, ksize=5)
            sobel = np.sqrt(sobelx**2 + sobely**2)
            sobel_score = np.mean(sobel)
            stats.append(sobel_score)
    
            self.frame_stats = stats
        except Exception as e:
            _logger.warning(f"Failed to compute frame stats: {str(e)}")
            self.frame_stats = [-1, -1, -1, -1, -1, -1]

    def get_frame(self):
        """
        Get the most recent camera frame with metadata
        
        Returns:
            FrameData object containing the frame, frame_id, and timestamp
            or None if no frame is available
        """
        if self.current_frame is None:
            return None
            
        frame_data = FrameData(self.current_frame.copy(), self.frame_id, self.prev_frame_time)
        return frame_data
    
    def enable_apriltags(self):
        """Initialize AprilTag detector for this camera"""
        if self.camera_params is None:
            _logger.warning(f"Cannot enable AprilTag detection for {self.name}: camera parameters not available")
            return False
            
        self.apriltag_detector = AprilTagDetector(camera_params=self.camera_params)
        _logger.info(f"Enabled AprilTag detection for {self.name}")
        return True
    
    def get_fov(self):
        """
        Estimate the horizontal and vertical field of view in degrees
        
        Uses camera intrinsic parameters to calculate FoV
        
        Returns:
            tuple: (horizontal_fov, vertical_fov) in degrees
            or (None, None) if camera parameters aren't available
        """
        if self.camera_params is None or len(self.camera_params) < 4:
            _logger.warning(f"Cannot calculate FoV for {self.name}: camera parameters not available")
            return None, None
            
        fx, fy, cx, cy = self.camera_params
        fov_x = 2 * np.arctan(self.width / (2 * fx)) * 180 / np.pi
        fov_y = 2 * np.arctan(self.height / (2 * fy)) * 180 / np.pi
        return fov_x, fov_y
    
    def get_exposure(self):
        """Get current camera exposure setting"""
        try:
            return self.cap.get(cv2.CAP_PROP_EXPOSURE)
        except:
            return None
    
    def get_gain(self):
        """Get current camera gain setting"""
        try:
            return self.cap.get(cv2.CAP_PROP_GAIN)
        except:
            return None
    
    def get_frame_stats(self):
        """
        Get image quality statistics for the most recent frame
        
        Returns:
            list: [min, max, mean, std, noise_estimate, edge_content]
            Returns [-1,...] if stats are disabled or unavailable
        """
        if not self.stats_enabled:
            return [-1, -1, -1, -1, -1, -1]
        return self.frame_stats
    
    def release(self):
        """Release the camera resources"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
            _logger.info(f"Released camera resources for {self.name}")

    def update_network_tables(self, tables):
        """Update network tables with current camera data"""
        tables.setDouble(f"sensors3140/camera{self.camera_id}/frame_id", self.frame_id)
        tables.setDouble(f"sensors3140/camera{self.camera_id}/timestamp", self.prev_frame_time)
        tables.setDouble(f"sensors3140/camera{self.camera_id}/fps_current", self.current_fps)
        tables.setDouble(f"sensors3140/camera{self.camera_id}/fps_ave", self.average_fps)
        tables.setDouble(f"sensors3140/camera{self.camera_id}/exposure", self.get_exposure())
        tables.setDouble(f"sensors3140/camera{self.camera_id}/gain", self.get_gain())
        tables.setDoubleArray(f"sensors3140/camera{self.camera_id}/frame_stats", self.get_frame_stats())

    def initialize_network_tables(self, tables):
        """Initialize network tables with camera configuration data"""
        tables.setDouble(f"sensors3140/camera{self.camera_id}/fps", self.fps_target)
        tables.setDoubleArray(f"sensors3140/camera{self.camera_id}/parameters", self.camera_params)
        tables.setDoubleArray(f"sensors3140/camera{self.camera_id}/distortion", self.dist_coeffs)
        tables.setDoubleArray(f"sensors3140/camera{self.camera_id}/fov", self.get_fov())
        tables.setDoubleArray(f"sensors3140/camera{self.camera_id}/size", [self.width, self.height])


def load_cameras_from_config_directory(config_dir: str = None) -> dict:
    """
    Load camera configurations from JSON files in a directory
    
    Args:
        config_dir: Directory containing camera configuration files
                   If None, uses ~/sensors3140/
    
    Returns:
        dict: Dictionary of Camera objects keyed by camera name
    """
    if config_dir is None:
        # default directory is ~/sensors3140/
        home_dir = os.path.expanduser("~")
        config_dir = os.path.join(home_dir, 'sensors3140')
        
    if not os.path.exists(config_dir):
        _logger.warning(f"Config directory {config_dir} does not exist")
        return {}

    cameras = {}
    camera_files = [f for f in os.listdir(config_dir) if f.startswith('camera') and f.endswith('.json')]
    
    if not camera_files:
        _logger.warning(f"No camera configuration files found in {config_dir}")
        return {}
        
    for file in camera_files:
        file_path = os.path.join(config_dir, file)
        _logger.info(f"Loading camera configuration from {file_path}")
        try:
            with open(file_path) as f:
                config = json.load(f)
                _logger.debug(f"Camera config: {config}")
                camera = create_camera_from_config(config)
                cameras[camera.name] = camera
        except Exception as e:
            _logger.error(f"Failed to load camera from {file_path}: {str(e)}")

    if not cameras:
        _logger.warning("No cameras were successfully loaded from configuration")
    else:
        _logger.info(f"Successfully loaded {len(cameras)} cameras: {', '.join(cameras.keys())}")
        
    return cameras

def create_camera_from_config(config: dict) -> Camera:
    """
    Create a Camera instance from a configuration dictionary
    
    Args:
        config: Dictionary containing camera configuration settings
               Must include name, camera_id, and may include frame_size and fps
               
    Returns:
        Camera: Initialized camera object
    """
    # Extract mandatory and optional parameters with defaults
    camera_name = config.get('name', f"Camera_{config.get('camera_id', 'unknown')}")
    camera_id = config.get('camera_id')
    
    if camera_id is None:
        raise ValueError(f"Missing camera_id in configuration for {camera_name}")
    
    # Extract resolution - can be specified as frame_size or as width/height
    if 'frame_size' in config:
        width, height = config.get('frame_size')
    else:
        width = config.get('width', 640)
        height = config.get('height', 480)
    
    fps = config.get('fps', 30)
    enable_stats = config.get('enable_stats', False)
    
    _logger.info(f"Creating camera {camera_name} with device ID {camera_id}")
    _logger.info(f"    Resolution: {width}x{height} @ {fps} FPS")
    
    # Create the camera with all available parameters
    camera_params = {
        'name': camera_name,
        'camera_id': camera_id,
        'width': width,
        'height': height,
        'fps': fps,
        'enable_stats': enable_stats
    }
    
    # Copy any additional parameters
    for key, value in config.items():
        if key not in camera_params:
            camera_params[key] = value

    try:
        camera = Camera(**camera_params)
        
        # Set camera calibration if available
        if 'camera_matrix' in config and 'distortion' in config:
            camera_matrix = np.array(config['camera_matrix'], dtype=np.float32).reshape((3, 3))
            distortion = np.array(config['distortion'], dtype=np.float32)
            camera.set_camera_calibration(camera_matrix, distortion)
            _logger.info(f"Loaded calibration for {camera_name}")
        
        return camera
    except Exception as e:
        _logger.error(f"Failed to create camera {camera_name}: {str(e)}")
        raise


if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        cameras = load_cameras_from_config_directory()
        if not cameras:
            _logger.warning("No cameras loaded. Starting with default camera (ID: 0)")
            try:
                default_camera = Camera(camera_id=0, name="Default")
                cameras = {"Default": default_camera}
            except Exception as e:
                _logger.error(f"Could not initialize default camera: {e}")
                exit(1)
        
        _logger.info(f"Starting camera preview with {len(cameras)} cameras")
        
        while True:
            for name, camera in cameras.items():
                frame_data = camera.get_frame()
                if frame_data is not None:
                    # Display FPS on the frame
                    frame = frame_data.frame.copy()
                    fps_text = f"FPS: {camera.average_fps:.1f}"
                    cv2.putText(frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               1, (0, 255, 0), 2)
                    cv2.imshow(name, frame)
    
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
    
        # Clean up
        for camera in cameras.values():
            camera.release()
        cv2.destroyAllWindows()
        
    except Exception as e:
        _logger.error(f"Error in main: {str(e)}")