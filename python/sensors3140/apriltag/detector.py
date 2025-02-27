import cv2
import apriltag
import numpy as np
from sensors3140.apriltag.maps.map import get_map, FieldMap
from sensors3140.tables.network_tables import NetworkTablesManager
import time


class AprilTagDetector:
    def __init__(self, camera_id, tag_family="tag36h11", camera_params=None, tag_size=.20638, game_id="2025-reefscape"):
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
        start_time = time.time()

        gray = cv2.cvtColor(frame_data.frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        detections = []
        tag_ids = []
        distances = []
        bearings = []
        azimuths = []
        decision_margin = []

        target = self.table.getInteger(f"sensors3140/apriltags/camera{self.camera_id}/target_id")

        best_camera_pose = None
        best_camera_translation = None
        best_camera_direction = None
        best_camera_tag_id = None
        best_camera_score = None

        for r in results:
            pose = self.detector.detection_pose(r, self.camera_params, self.tag_size)
            trans = pose[0][0:3, 3]
            dist = np.sqrt((trans**2).sum())
            bearing = np.arctan2(trans[0], trans[2]) * 180.0 / np.pi
            azimuth = np.arctan2(-trans[1], trans[2]) * 180.0 / np.pi
            decision_margin.append(r.decision_margin)

            detections.append({
                'id': r.tag_id,
                'center': r.center,
                'corners': r.corners,
                'distance': dist,
                'bearing': bearing,
                'azimuth': azimuth,
                'tag_rotation': pose[0][:3, :3],
                'tag_translation': pose[0][:3, 3],
                'pose': pose[0],
                'pose_decomposed': self.decompose_pose_matrix(pose[0]),
                'camera_params': self.camera_params,
            })

            tag_to_camera_transform = pose[0]
            detections[-1]['tag_pose'] = tag_to_camera_transform

            camera_to_tag_transform = np.linalg.inv(tag_to_camera_transform)
            detections[-1]['camera_pose'] = camera_to_tag_transform

            camera_origin_in_tag_space = np.dot(camera_to_tag_transform, np.array([[0], [0], [0], [1]]))
            camera_origin_in_tag_space = camera_origin_in_tag_space[:3] / camera_origin_in_tag_space[3]
            detections[-1]['camera_translation'] = camera_origin_in_tag_space

            if r.tag_id not in self.map_data.getAllTags():
                continue

            tag_to_field_transform = self.map_data.getTagTransform(r.tag_id)

            camera_origin_homogeneous = np.array([[0.0], [0.0], [0.0], [1.0]])
            camera_forward_homogeneous = np.array([[0.0], [0.0], [1.0], [1.0]])

            camera_origin_homogeneous = np.matmul(camera_to_tag_transform, camera_origin_homogeneous)
            camera_forward_homogeneous = np.matmul(camera_to_tag_transform, camera_forward_homogeneous)

            camera_origin_homogeneous = np.matmul(tag_to_field_transform, camera_origin_homogeneous)
            camera_forward_homogeneous = np.matmul(tag_to_field_transform, camera_forward_homogeneous)

            camera_position_field = camera_origin_homogeneous[:3] / camera_origin_homogeneous[3]
            camera_direction_field = camera_forward_homogeneous[:3] / camera_forward_homogeneous[3]

            camera_location_angle = np.arctan2(camera_to_tag_transform[1, 3], camera_to_tag_transform[0, 3])
            score = camera_location_angle

            detections[-1]['camera_location'] = camera_origin_homogeneous
            detections[-1]['camera_direction'] = camera_forward_homogeneous
            detections[-1]['camera_location_score'] = score

            if best_camera_pose is None or score > best_camera_score:
                best_camera_tag_id = r.tag_id
                best_camera_pose = camera_to_tag_transform
                best_camera_translation = camera_position_field
                best_camera_direction = camera_direction_field
                best_camera_score = score

            if r.tag_id == target:
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_distance", dist)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_bearing", bearing)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_azimuth", azimuth)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_timestamp", frame_data.timestamp)

            tag_ids.append(r.tag_id)
            distances.append(dist)
            bearings.append(bearing)
            azimuths.append(azimuth)

        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/timestamp", frame_data.timestamp)
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/frame_id", frame_data.frame_id)
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/count", len(detections))
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/ids", tag_ids)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/distances", distances)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/bearings", bearings)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/azimuths", azimuths)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/decision_margin", decision_margin)

        if len(decision_margin) > 0:
            average_decision_margin = sum(decision_margin) / len(decision_margin)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_decision_margin", average_decision_margin)
        else:
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_decision_margin", 0)

        if best_camera_pose is not None:
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/best_camera_pose", best_camera_pose.flatten())
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/best_camera_translation", best_camera_translation.flatten())
            self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/best_camera_direction", best_camera_direction.flatten())
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/best_camera_pose_tag", best_camera_tag_id)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/best_camera_pose_score", best_camera_score)

        end_time = time.time()
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/processing_time", end_time - start_time)

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

