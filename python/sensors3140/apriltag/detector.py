import cv2
import apriltag
import numpy as np
np.set_printoptions(precision=3, suppress=True)
from sensors3140.apriltag.maps.map import get_map, FieldMap
from sensors3140.tables.network_tables import NetworkTablesManager
import time

import ctypes


#dst = cv2.undistortPoints(src, cameraMatrix, distCoeffs)

def local_detection_pose(detector, detection, camera_params, dist_coeffs, tag_size=.1651, z_sign=1):

    fx, fy, cx, cy = [ ctypes.c_double(c) for c in camera_params ]
    
    H = detector.libc.matd_create(3, 3)
    arr = apriltag._matd_get_array(H)
    arr[:] = detection.homography
    corners = detection.corners.flatten().astype(np.float64).reshape(-1, 1, 2)
    corners_orig = corners.copy()

    camera_matrix = np.eye(3, 3, dtype=np.float64)
    camera_matrix[0, 0] = camera_params[0]
    camera_matrix[1, 1] = camera_params[1]
    camera_matrix[0, 2] = camera_params[2]
    camera_matrix[1, 2] = camera_params[3]

    corners = cv2.undistortPoints(corners, camera_matrix, dist_coeffs,P=camera_matrix).flatten()

    print('Corners')
    print(corners.flatten())
    print(corners_orig.flatten())

    dptr = ctypes.POINTER(ctypes.c_double)

    corners = corners.ctypes.data_as(dptr)


    init_error = ctypes.c_double(0)
    final_error = ctypes.c_double(0)
    
    Mptr = detector.libc.pose_from_homography(H, fx, fy, cx, cy,
                                            ctypes.c_double(tag_size),
                                            ctypes.c_double(z_sign),
                                            corners,
                                            dptr(init_error),
                                            dptr(final_error))

    M = apriltag._matd_get_array(Mptr).copy()
    detector.libc.matd_destroy(H)
    detector.libc.matd_destroy(Mptr)

    return M, init_error.value, final_error.value



class AprilTagDetector:
    def __init__(self, camera_id, tag_family="tag36h11", camera_params=None, dist_coeff=None, tag_size=.1651, game_id="2025-reefscape"):
        self.detector_options = apriltag.DetectorOptions(families=tag_family)

        # options = apriltag.Detectoroptions(families='tag36h11',
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
        self.load_map(game_id)

        self.detected_tags = set()

        self.table = NetworkTablesManager()
        self.table.setString(f"sensors3140/apriltags/camera{camera_id}/family", tag_family)
        self.table.setInteger(f"sensors3140/apriltags/camera{camera_id}/target_id", -1)

        print(f"Created AprilTagDetector for camera {camera_id}")


    def load_map(self, game_id):
        self.map_data = get_map(game_id)
        self.map_data: FieldMap


    def __call__(self, frame_data):
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
            A list of dictionaries, each containing information about a detected AprilTag:
            - id: Tag ID
            - center: Center point of the tag in image coordinates
            - corners: Corner points of the tag in image coordinates
            - distance: Distance to the tag in meters
            - bearing: Horizontal angle to the tag in degrees
            - azimuth: Vertical angle to the tag in degrees
            - tag_rotation: 3x3 rotation matrix representing tag orientation
            - tag_translation: Tag position in camera frame
            - pose: Full 4x4 transformation matrix from camera to tag
            - pose_decomposed: Decomposed pose (translation and rotation)
            - camera_params: Camera parameters used for pose estimation
            - decision_margin: Confidence score of the detection
            - tag_pose: Transformation from camera to tag
            - camera_pose: Transformation from tag to camera
            - camera_translation: Camera position in tag space
            For tags in the map, additional fields:
            - camera_location: Camera position in field coordinates (homogeneous)
            - camera_direction: Camera forward direction in field coordinates (homogeneous)
            - camera_location_score: Score used to select best tag for camera positioning
        """
        start_time = time.time()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame_data.frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        # Clear the detected tags set
        detections = []
        tag_ids = []
        distances = []
        bearings = []
        azimuths = []
        decision_margins = []
        camera_positions = []
        camera_directions = []
        camera_angles = []

        # Get the target tag ID
        target = self.table.getInteger(f"sensors3140/apriltags/camera{self.camera_id}/target_id")

        # Setup variables to store the best camera position
        best_camera_to_tag_transform = None
        best_camera_position_field = None
        best_camera_direction_field = None
        best_camera_tag_id = None
        best_camera_location_angle_score = None

        # Process each detected tag
        for r in results:
            # Get the tag pose in camera frame
            tag_pose_matrices = local_detection_pose(self.detector, r, self.camera_params,self.dist_coeff, self.tag_size)
            tag_translation_camera_frame = tag_pose_matrices[0][0:3, 3]
            tag_distance = np.sqrt((tag_translation_camera_frame**2).sum())
            tag_bearing = np.arctan2(tag_translation_camera_frame[0], tag_translation_camera_frame[2]) * 180.0 / np.pi
            tag_azimuth = np.arctan2(-tag_translation_camera_frame[1], tag_translation_camera_frame[2]) * 180.0 / np.pi
            
            print(f"Tag {r.tag_id}: distance={tag_distance:.2f}m")
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

            # Calculate the camera position in field coordinates
            tag_to_camera_transform = tag_pose_matrices[0]
            detections[-1]['tag_pose'] = tag_to_camera_transform

            # Calculate the tag position in camera coordinates
            camera_to_tag_transform = np.linalg.inv(tag_to_camera_transform)
            detections[-1]['camera_pose'] = camera_to_tag_transform

            # Calculate the camera position in tag space
            camera_origin_in_tag_space = np.dot(camera_to_tag_transform, np.array([[0], [0], [0], [1]]))
            camera_origin_in_tag_space = camera_origin_in_tag_space[:3] / camera_origin_in_tag_space[3]
            detections[-1]['camera_translation'] = camera_origin_in_tag_space

            # Skip tags not in the map
            if r.tag_id not in self.map_data.getAllTags():
                continue

            # Get the tag-to-field transform
            tag_to_field_transform = self.map_data.getTagTransform(r.tag_id)

            # Calculate the camera position in field coordinates
            camera_origin_homogeneous = np.array([[0.0], [0.0], [0.0], [1.0]])
            camera_forward_homogeneous = np.array([[0.0], [0.0], [1.0], [1.0]])

            camera_origin_homogeneous = np.matmul(camera_to_tag_transform, camera_origin_homogeneous)
            camera_forward_homogeneous = np.matmul(camera_to_tag_transform, camera_forward_homogeneous)

            camera_origin_homogeneous = np.matmul(tag_to_field_transform, camera_origin_homogeneous)
            camera_forward_homogeneous = np.matmul(tag_to_field_transform, camera_forward_homogeneous)

            camera_position_field = camera_origin_homogeneous[:3] / camera_origin_homogeneous[3]
            camera_direction_field = camera_forward_homogeneous[:3] / camera_forward_homogeneous[3]

            # Calculate the angle of the camera in field coordinates
            camera_location_angle = np.arctan2(camera_to_tag_transform[1, 3], camera_to_tag_transform[0, 3])
            camera_location_angle_score = camera_location_angle

            # Add the camera position to the detection
            detections[-1]['camera_location'] = camera_origin_homogeneous
            detections[-1]['camera_direction'] = camera_forward_homogeneous
            detections[-1]['camera_location_score'] = camera_location_angle_score

            # Update the best camera position
            if best_camera_to_tag_transform is None or camera_location_angle_score > best_camera_location_angle_score:
                best_camera_tag_id = r.tag_id
                best_camera_to_tag_transform = camera_to_tag_transform
                best_camera_position_field = camera_position_field
                best_camera_direction_field = camera_direction_field
                best_camera_location_angle_score = camera_location_angle_score

            # Update the detected tags set
            if r.tag_id == target:
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_distance", tag_distance)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_bearing", tag_bearing)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_azimuth", tag_azimuth)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_timestamp", frame_data.timestamp)

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
        if best_camera_to_tag_transform is not None:
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_position", best_camera_position_field.flatten())
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/camera_direction", best_camera_direction_field.flatten())
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_tag", best_camera_tag_id)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/camera_position_score", best_camera_location_angle_score)

        # publish the processing time
        end_time = time.time()
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/processing_time", end_time - start_time)

        # store the detections
        self.detections = detections

        return detections
    
    def get_detected_tags(self):
        return self.detections
    
    def decompose_pose_matrix(self, pose_matrix: np.ndarray) -> dict[str, float]:
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

