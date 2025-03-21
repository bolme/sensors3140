import cv2
import apriltag
import numpy as np
import logging
np.set_printoptions(precision=3, suppress=True)
from sensors3140.maps import get_map, FieldMap
from sensors3140.tables.network_tables import NetworkTablesManager
import time
import ctypes
from typing import List, Dict, Tuple, Any

POSITION_QUALITY_THRESHOLD = 0.1
POSITION_QUALITY_MIN_ANGLE = np.pi*10./180. # seems like a good value
POSITION_QUALITY_MAX_ANGLE = np.pi*70./180. # seems like a good value
DISTANCE_HALF_LIMIT = 4.0 # meters
DISTANCE_LIMIT = 5.0 # meters
SINGLE_TAG_DISTANCE_LIMIT = 2.0 # meters

_logger = logging.getLogger(__name__)

def _local_detection_pose(detector: apriltag.Detector, detection: apriltag.Detection, camera_params: List[float], dist_coeffs: List[float], tag_size: float = .1651, z_sign: int = 1) -> Tuple[np.ndarray, float, float]:
    """
    Calculate the pose of an AprilTag in the camera reference frame
    
    This function accounts for lens distortion by undistorting the corners
    before estimating the pose.
    
    Args:
        detector: AprilTag detector instance
        detection: AprilTag detection object
        camera_params: Camera intrinsic parameters [fx, fy, cx, cy]
        dist_coeffs: Lens distortion coefficients
        tag_size: Size of the tag in meters
        z_sign: Sign of the Z axis (1 or -1)
        
    Returns:
        tuple: (pose_matrix, initial_error, final_error)
    """
    fx, fy, cx, cy = [ ctypes.c_double(c) for c in camera_params ]
    
    # Create a homography matrix from the detection
    H = detector.libc.matd_create(3, 3)
    arr = apriltag._matd_get_array(H)
    arr[:] = detection.homography
    
    # Undistort corners for more accurate pose estimation
    corners = detection.corners.flatten().astype(np.float64).reshape(-1, 1, 2)

    # Create camera matrix from intrinsic parameters
    camera_matrix = np.eye(3, 3, dtype=np.float64)
    camera_matrix[0, 0] = camera_params[0]
    camera_matrix[1, 1] = camera_params[1]
    camera_matrix[0, 2] = camera_params[2]
    camera_matrix[1, 2] = camera_params[3]

    # Undistort corners
    corners = cv2.undistortPoints(corners, camera_matrix, dist_coeffs,P=camera_matrix).flatten()

    # Debug log the corner positions (only at debug level to avoid excessive logging)
    _logger.debug(f"Corners after undistortion: {corners.flatten()}")

    # Convert corners to C pointer for apriltag library
    dptr = ctypes.POINTER(ctypes.c_double)
    corners = corners.ctypes.data_as(dptr)

    # Variables to receive error values
    init_error = ctypes.c_double(0)
    final_error = ctypes.c_double(0)
    
    # Call into apriltag library to estimate pose
    Mptr = detector.libc.pose_from_homography(H, fx, fy, cx, cy,
                                            ctypes.c_double(tag_size),
                                            ctypes.c_double(z_sign),
                                            corners,
                                            dptr(init_error),
                                            dptr(final_error))

    # Convert the result to numpy array and clean up
    M = apriltag._matd_get_array(Mptr).copy()
    detector.libc.matd_destroy(H)
    detector.libc.matd_destroy(Mptr)

    return M, init_error.value, final_error.value

def _compute_camera_pose(camera_params, camera_distortion, image_points, field_points) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute the camera pose from image and field points
    
    Args:
        camera_matrix: Camera intrinsic matrix
        camera_distortion: Lens distortion coefficients
        image_points: Image points
        field_points: Field points
        
    Returns:
        tuple: (rvec, tvec)
    """
    # Convert the field points to 3D
    field_points = np.array(field_points, dtype=np.float32)
    
    # Convert camera params to camera matrix
    camera_matrix = np.array([
        [camera_params[0], 0, camera_params[2]],
        [0, camera_params[1], camera_params[3]],
        [0, 0, 1]
    ], dtype=np.float32)

    # Find the camera pose
    success, rvec, tvec = cv2.solvePnP(field_points, image_points, camera_matrix, camera_distortion)
    
    # Camera location is the negative of the rotation matrix times the translation vector
    camera_location_3d = -np.dot(cv2.Rodrigues(rvec)[0].T, tvec)
    camera_front_vector = np.dot(cv2.Rodrigues(rvec)[0].T, np.array([[0], [0], [1]]))
    camera_right_vector = np.dot(cv2.Rodrigues(rvec)[0].T, np.array([[1], [0], [0]]))
    camera_up_vector = np.dot(cv2.Rodrigues(rvec)[0].T, np.array([[0], [-1], [0]])) # negative y-axis is up

    return camera_location_3d, camera_front_vector, camera_right_vector, camera_up_vector



class AprilTagDetector:
    """
    AprilTag detector that processes camera frames and publishes results to NetworkTables
    
    This class handles tag detection, pose estimation, and field positioning based on tags.
    """
    def __init__(self, camera_id: int, tag_family: str = "tag36h11", camera_params: List[float] = None, dist_coeff: List[float] = None, tag_size: float = .1651, game_id: str = "2025-reefscape"):
        """
        Initialize the AprilTag detector
        
        Args:
            camera_id: ID of the camera
            tag_family: AprilTag family to detect (default: tag36h11)
            camera_params: Camera intrinsic parameters [fx, fy, cx, cy]
            dist_coeff: Lens distortion coefficients
            tag_size: Size of tags in meters
            game_id: ID of the game field map to use
        """
        self.detector_options = apriltag.DetectorOptions(families=tag_family)

        # Additional detector options can be set if needed:
        # options = apriltag.DetectorOptions(families='tag36h11',
        #                          border=1,
        #                          nthreads=4,
        #                          quad_decimate=1.0,
        #                          quad_blur=0.0,
        #                          refine_edges=True,
        #                          refine_decode=False,
        #                          refine_pose=False,
        #                          debug=False,
        #                          quad_contours=True)

        self.detector = apriltag.Detector(self.detector_options)
        self.camera_params = camera_params
        self.dist_coeff = dist_coeff
        self.tag_size = tag_size
        self.camera_id = camera_id
        self.detections = []
        self.decision_quality_average = None
        
        # For periodic logging
        self.last_log_time = time.time()
        self.tags_detected = 0
        
        # For 1-second statistics logging
        self.last_stats_time = time.time()
        self.frames_processed = 0
        self.second_tags_detected = 0
        self.min_tag_distance = float('inf')
        self.max_tag_distance = 0.0
        
        # Load the field map with tag positions
        self.load_map(game_id)

        self.detected_tags = set()

        # Initialize NetworkTables entries
        self.table = NetworkTablesManager()
        self.table.setString(f"sensors3140/apriltags/camera{camera_id}/family", tag_family)
        self.table.setInteger(f"sensors3140/apriltags/camera{camera_id}/target_id", -1)

        _logger.info(f"Created AprilTagDetector for camera {camera_id}")


    def load_map(self, game_id: str) -> None:
        """Load the field map containing AprilTag positions"""
        self.map_data = get_map(game_id)
        self.map_data: FieldMap
        _logger.info(f"Loaded field map '{game_id}' with {len(self.map_data.get_all_tags())} tags")


    def __call__(self, frame_data: Any) -> List[Dict[str, Any]]:
        """
        Process a frame to detect AprilTags.
        This method takes a frame, detects AprilTags in it, calculates various metrics like 
        distance, bearing, azimuth for each tag, and determines the camera's position in field 
        coordinates based on detected tags. Results are published to NetworkTables.
        
        Parameters
        ----------
        frame_data : FrameData
            An object containing the frame image and metadata like timestamp and frame_id.
        
        Returns
        -------
        list
            A list of dictionaries, each containing information about a detected AprilTag.
        """
        start_time = time.time()
        
        # Increment frame counter
        self.frames_processed += 1

        # Check if frame is valid before processing
        if frame_data is None or frame_data.frame is None or frame_data.frame.size == 0:
            _logger.warning(f"Warning: Received empty frame in camera {self.camera_id}")
            # Publish empty results
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/timestamp", 
                                frame_data.timestamp if frame_data else time.time())
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/count", 0)
            return []

        # Convert the frame to grayscale for tag detection
        gray = cv2.cvtColor(frame_data.frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        
        # Increment detector stats
        self.tags_detected += len(results)
        self.second_tags_detected += len(results)

        # Reset min/max distances when starting a new frame batch
        if time.time() - self.last_stats_time > 1.0:
            self.min_tag_distance = float('inf')
            self.max_tag_distance = 0.0

        # Clear previous detections
        detections = []
        tag_ids = []
        distances = []
        bearings = []
        azimuths = []
        decision_margins = []
        camera_positions = []
        camera_directions = []
        camera_angles = []

        all_detected_image_corners = []
        all_detected_field_corners = []

        # Get the target tag ID from NetworkTables
        target = self.table.getInteger(f"sensors3140/apriltags/camera{self.camera_id}/target_id")

        # Setup variables to store the best camera position
        best_camera_to_tag_transform = None
        best_camera_position_field = None
        best_camera_direction_field = None
        best_camera_tag_id = None
        best_location_quality = None
        best_camera_distance = None
        best_camera_tag_size = None

        # Process each detected tag
        for r in results:
            

            # Get the detected corners for this tag
            corners = r.corners.astype(np.float32)
            all_detected_image_corners.append(corners)

            # Get the field corners for this tag
            field_corners = self.map_data.get_tag_corners_in_field_coordinates(r.tag_id, tag_size=self.tag_size).astype(np.float32)
            all_detected_field_corners.append(field_corners)

            #print(f"Tag {r.tag_id} image corners: {corners} {corners.shape}")
            #print(f"Tag {r.tag_id} field corners: {field_corners} {field_corners.shape}")

            # Compute the camera location and direction in field coordinates
            camera_location, camera_front, camera_right, camera_up = _compute_camera_pose(
                np.array(self.camera_params[:4]), np.array(self.dist_coeff), corners, field_corners
            )
            #print(f"Camera location: {camera_location.flatten()}")
            #print(f"Camera front: {camera_front.flatten()}")
            #print(f"Camera up: {camera_up.flatten()}")

            camera_position_field = camera_location.flatten()
            camera_direction_field = camera_position_field+camera_front.flatten()

            # Get the tag pose in camera frame
            tag_pose_matrices = _local_detection_pose(self.detector, r, self.camera_params,self.dist_coeff, self.tag_size)
            tag_translation_camera_frame = tag_pose_matrices[0][0:3, 3]
            tag_distance = np.sqrt((tag_translation_camera_frame**2).sum())

            # limit the overall distance
            if tag_distance > SINGLE_TAG_DISTANCE_LIMIT:
                continue
            
            # Track min/max distances
            self.min_tag_distance = min(self.min_tag_distance, tag_distance)
            self.max_tag_distance = max(self.max_tag_distance, tag_distance)
            
            tag_bearing = np.arctan2(tag_translation_camera_frame[0], tag_translation_camera_frame[2]) * 180.0 / np.pi
            tag_azimuth = np.arctan2(-tag_translation_camera_frame[1], tag_translation_camera_frame[2]) * 180.0 / np.pi
            
            _logger.debug(f"Tag {r.tag_id}: distance={tag_distance:.2f}m, bearing={tag_bearing:.1f}°, azimuth={tag_azimuth:.1f}°")
                        
            # Add the detection to the list
            detections.append({
                'id': r.tag_id,
                'center': r.center,
                'corners': r.corners,
                'distance': tag_distance,
                'bearing': tag_bearing,
                'azimuth': tag_azimuth,
                'tag_rotation': tag_pose_matrices[0][:3, :3],
                'tag_translation': tag_pose_matrices[0][:3, 3],
                'pose': tag_pose_matrices[0],
                'pose_decomposed': self.decompose_pose_matrix(tag_pose_matrices[0]),
                'camera_params': self.camera_params,
                'decision_margin': r.decision_margin,
            })

            # Calculate transformations between camera and tag
            tag_to_camera_transform = tag_pose_matrices[0]
            detections[-1]['tag_pose'] = tag_to_camera_transform

            # Calculate the tag position in camera coordinates
            camera_to_tag_transform = np.linalg.inv(tag_to_camera_transform)
            detections[-1]['camera_pose'] = camera_to_tag_transform

            # Calculate the camera position in tag space
            camera_origin_in_tag_space = np.dot(camera_to_tag_transform, np.array([[0], [0], [0], [1]]))
            camera_origin_in_tag_space = camera_origin_in_tag_space[:3] / camera_origin_in_tag_space[3]
            detections[-1]['camera_translation'] = camera_origin_in_tag_space

            # Skip tags not in the field map
            if r.tag_id not in self.map_data.get_all_tags():
                continue

            # Get the tag position in field coordinates
            tag_to_field_transform = self.map_data.get_tag_transform(r.tag_id)

            # Calculate the camera position in field coordinates
            camera_origin_homogeneous = np.array([[0.0], [0.0], [0.0], [1.0]])
            camera_forward_homogeneous = np.array([[0.0], [0.0], [1.0], [1.0]])

            # Transform camera origin and forward direction to tag coordinates
            camera_origin_homogeneous = np.matmul(camera_to_tag_transform, camera_origin_homogeneous)
            camera_forward_homogeneous = np.matmul(camera_to_tag_transform, camera_forward_homogeneous)

            # Transform from tag to field coordinates
            camera_origin_homogeneous = np.matmul(tag_to_field_transform, camera_origin_homogeneous)
            camera_forward_homogeneous = np.matmul(tag_to_field_transform, camera_forward_homogeneous)

            #camera_position_field = camera_origin_homogeneous[:3] / camera_origin_homogeneous[3]
            #camera_direction_field = camera_forward_homogeneous[:3] / camera_forward_homogeneous[3]

            # Score this tag for camera positioning (favoring tags seen head-on)
            camera_location_angle = np.arctan2(camera_to_tag_transform[1, 3], camera_to_tag_transform[0, 3])

            # Add camera position info to the detection
            detections[-1]['camera_location'] = camera_origin_homogeneous
            detections[-1]['camera_direction'] = camera_forward_homogeneous

            # Get the tag location and direction in field coordinates
            tag_location = np.matmul(tag_to_field_transform, np.array([[0.0], [0.0], [0.0], [1.0]]))
            tag_direction = np.matmul(tag_to_field_transform, np.array([[0.0], [0.0], [-1.0], [1.0]]))
                                      
            # If there is a ray in the tag direction find the closest point on the ray to the camera
            
            #subtract the tag location from the camera location
            #print("Camera position field: ", camera_position_field)
            #print("Tag location: ", tag_location[:3])
            camera_to_tag = camera_position_field - tag_location[:3].flatten()
            #subtract the tag location from the tag direction
            tag_to_tag_direction = tag_direction[:3].flatten() - tag_location[:3].flatten()
            # Find the angle in radians between the camera_to_tag and tag_to_tag_direction vectors
            tag_to_camera_angle_rad = np.arccos(np.dot(camera_to_tag.flatten(), tag_to_tag_direction.flatten()) / (np.linalg.norm(camera_to_tag) * np.linalg.norm(tag_to_tag_direction)))
            
            # camera to tag distance
            distance = np.linalg.norm(camera_to_tag)

            # givin the corner points calculate the area in square pixels
            #print(r.corners, type(r.corners), r.corners.shape)
            tag_size_squared = cv2.contourArea(r.corners.astype(np.float32))
            tag_size = np.sqrt(tag_size_squared)
            #print("Tag size: ", tag_size)

            # compute the angle quality
            angle_quality = np.cos(tag_to_camera_angle_rad)*(tag_to_camera_angle_rad>POSITION_QUALITY_MIN_ANGLE)
            
            # This is a sigmoid
            distance_quality = 1/(1+np.exp(0.5*(distance-DISTANCE_HALF_LIMIT)))
            
            location_quality = (angle_quality > POSITION_QUALITY_MIN_ANGLE) and (angle_quality < POSITION_QUALITY_MAX_ANGLE) and (distance < SINGLE_TAG_DISTANCE_LIMIT)

            # Update the best camera position if this tag is better
            if best_camera_distance is None:
                best_camera_distance = distance
            if location_quality and (best_camera_distance <= distance):
                print("Location Found.")
                best_camera_tag_id = r.tag_id
                best_camera_to_tag_transform = camera_to_tag_transform
                best_camera_position_field = camera_position_field
                best_camera_direction_field = camera_direction_field
                best_location_quality = True
                best_camera_distance = distance
                best_camera_tag_size = tag_size

            # Process target tag
            if r.tag_id == target:
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_distance", tag_distance)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_bearing", tag_bearing)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_azimuth", tag_azimuth)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_area", tag_size_squared)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_size", tag_size)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_tag_to_camera_angle", tag_to_camera_angle_rad)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_angle_quality", angle_quality)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_timestamp", frame_data.timestamp)

                # Get the tag location and direction in field coordinates
                tag_location = np.matmul(tag_to_field_transform, np.array([[0.0], [0.0], [0.0], [1.0]]))
                tag_direction = np.matmul(tag_to_field_transform, np.array([[0.0], [0.0], [-1.0], [1.0]]))
                tag_left = np.matmul(tag_to_field_transform, np.array([[-0.165], [0.0], [0.0], [1.0]]))
                tag_right = np.matmul(tag_to_field_transform, np.array([[0.165], [0.0], [0.0], [1.0]]))

                self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/target_field_location", tag_location.flatten())
                self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/target_field_direction", tag_direction.flatten())
                self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/target_left", tag_left.flatten())
                self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/target_right", tag_right.flatten())

            # Add the detected tag to the set
            tag_ids.append(r.tag_id)
            distances.append(tag_distance)
            bearings.append(tag_bearing)
            azimuths.append(tag_azimuth)
            camera_positions.append(camera_position_field)
            camera_directions.append(camera_direction_field)
            camera_angle_field = np.arctan2(camera_direction_field[1], camera_direction_field[0])*180.0/np.pi
            camera_angles.append(camera_angle_field)
            decision_margins.append(r.decision_margin)


        # Compute the best camera pose based on all corners
        n_tags_detected = len(all_detected_image_corners)
        best_compution_method = "single_tag"
        if n_tags_detected > 1:
            #print("All detected image corners: ", all_detected_image_corners)
            #print("All detected field corners: ", all_detected_field_corners)
            all_detected_image_corners = np.array(all_detected_image_corners).reshape(-1, 2)
            all_detected_field_corners = np.array(all_detected_field_corners).reshape(-1, 3)
            #print("All detected image corners: ", all_detected_image_corners)
            #print("All detected field corners: ", all_detected_field_corners)
            #print("All detected image corners: ", all_detected_image_corners.shape)
            #print("All detected field corners: ", all_detected_field_corners.shape)

            # Compute the camera pose based on all detected corners
            best_camera_location, best_camera_front, camera_right, best_camera_up = _compute_camera_pose(
                np.array(self.camera_params[:4]), np.array(self.dist_coeff), all_detected_image_corners, all_detected_field_corners
                )
            
            best_camera_position_field = best_camera_location.flatten()
            best_camera_direction_field = best_camera_position_field+best_camera_front.flatten()
            
            best_compution_method = "multitag"

        # Publish the detections to NetworkTables
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/timestamp", frame_data.timestamp)
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/frame_id", frame_data.frame_id)
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/count", len(detections))
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/ids", tag_ids)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/distances", distances)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/bearings", bearings)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/azimuths", azimuths)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/decision_margin", decision_margins)

        # flatten the camera_positions and camera_directions arrays
        camera_positions = np.array(camera_positions).flatten()
        camera_directions = np.array(camera_directions).flatten()

        # get the camera angle in field coordinates
        camera_angles = np.array(camera_angles)

        # publish the camera positions and directions
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_positions", camera_positions)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_directions", camera_directions)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_angles", camera_angles)

        # calculate the average decision margin
        if len(decision_margins) > 0:
            average_decision_margin = sum(decision_margins) / len(decision_margins)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_decision_margin", average_decision_margin)
        else:
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_decision_margin", 0)

        # publish the best camera position
        ave_distance = np.mean(distances)
        if False and best_compution_method == 'multitag':
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_position", best_camera_position_field.flatten())
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_direction", best_camera_direction_field.flatten())
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_tag", best_camera_tag_id)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_timestamp", frame_data.timestamp)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_quailty", best_location_quality)
            self.table.setString(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_method", best_compution_method)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_distance", best_camera_distance)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_tag_size", best_camera_tag_size)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_detected", 1.0)
        
        elif best_location_quality:
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_position", best_camera_position_field.flatten())
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_direction", best_camera_direction_field.flatten())
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_tag", best_camera_tag_id)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_timestamp", frame_data.timestamp)
            #self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_quailty", best_location_quality)
            self.table.setString(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_method", best_compution_method)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_distance", best_camera_distance)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_tag_size", best_camera_tag_size)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_detected", 1.0)
        else:
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_position", [0.0, 0.0, 0.0])
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_direction", [0.0, 0.0, 0.0])
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_tag", -1)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_timestamp", frame_data.timestamp)
            #self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_quailty", best_location_quality)
            self.table.setString(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_method", 'none')
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_distance", -1.0)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_tag_size", -1.0)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_detected", 0.0)


        # publish the processing time
        end_time = time.time()
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/processing_time", end_time - start_time)

        # store the detections
        self.detections = detections

        # Periodic logging
        if time.time() - self.last_log_time > 60:
            _logger.info(f"Camera {self.camera_id}: Detected {self.tags_detected} tags in the last minute")
            self.tags_detected = 0
            self.last_log_time = time.time()
            
        # Statistics logging (every 1 second)
        current_time = time.time()
        if current_time - self.last_stats_time > 1.0:
            elapsed = current_time - self.last_stats_time
            avg_detections = self.second_tags_detected / self.frames_processed if self.frames_processed > 0 else 0
            
            # Format distance info, handling the case when no tags are detected
            distance_info = ""
            if self.second_tags_detected > 0:
                distance_info = f"dist=[{self.min_tag_distance:.2f}, {self.max_tag_distance:.2f}]m "
            
            _logger.info(
                f"Camera {self.camera_id} stats: {self.frames_processed} frames in {elapsed:.2f}s "
                f"{self.second_tags_detected} tags "
                f"({avg_detections:.2f} avg) {distance_info}"
            )
            # Reset the counters
            self.frames_processed = 0
            self.second_tags_detected = 0
            self.min_tag_distance = float('inf')
            self.max_tag_distance = 0.0
            self.last_stats_time = current_time

        return detections, best_camera_position_field, best_camera_direction_field
    
    def get_detected_tags(self) -> List[Dict[str, Any]]:
        return self.detections
    
    def decompose_pose_matrix(self, pose_matrix: np.ndarray) -> Dict[str, float]:
        """
        Decompose 4x4 pose matrix into translation and rotation components
        Returns dictionary with x,y,z translations and roll,pitch,yaw angles in degrees
        """
        # Extract translation vector
        translation = pose_matrix[:3, 3]
        
        # Extract rotation matrix
        rotation = pose_matrix[:3, :3]
        
        # Convert rotation matrix to euler angles
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], np.sqrt(rotation[2, 1]**2 + rotation[2, 2]**2))
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        
        return {
            "x": float(translation[0]),
            "y": float(translation[1]),
            "z": float(translation[2]),
            "roll": float(np.degrees(roll)),
            "pitch": float(np.degrees(pitch)),
            "yaw": float(np.degrees(yaw))
        }

