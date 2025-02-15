import cv2
import apriltag
import numpy as np
from sensors3140.apriltag.maps.map import load_apriltags
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

        self.load_map(game_id)

        self.detected_tags = set()

        self.table = NetworkTablesManager()
        self.table.setString(f"sensors3140/apriltags/camera{camera_id}/family", tag_family)
        self.table.setInteger(f"sensors3140/apriltags/camera{camera_id}/target_id", -1)

        print(f"Created AprilTagDetector for camera {camera_id}")


    def load_map(self, game_id):
        self.map_data = load_apriltags(game_id)

    def __call__(self, frame_data):
        start_time = time.time()
        frame = frame_data.frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)
        # Results format:
        # [
        #   DetectionBase(
        #       tag_family='tag36h11', 
        #       tag_id=2, 
        #       hamming=0, 
        #       goodness=0.0, 
        #       decision_margin=98.58241271972656, 
        #       homography=array([[ -1.41302664e-01,   1.08428082e+00,   1.67512900e+01], [ -8.75899366e-01,   1.50245469e-01,   1.70532040e+00], [ -4.89183533e-04,   2.12210247e-03,   6.02052342e-02]]), 
        #       center=array([ 278.23643912,   28.32511859]), 
        #       corners=array([[ 269.8939209 ,   41.50381088], [ 269.57183838,   11.79248142], [ 286.1383667 ,   15.84242821], [ 286.18066406,   43.48323059]])), ...
        
        detections = []
        tag_ids = []
        distances = []
        bearings = []
        azimuths = []
        target = self.table.getInteger(f"sensors3140/apriltags/camera{self.camera_id}/target_id")
        #print("Target:",target)
        current_detected_tags = set()
        for r in results:
            # camera param format: [fx, fy, cx, cy]
            pose = self.detector.detection_pose(r, self.camera_params, self.tag_size)
            trans = pose[0][0:3, 3]
            dist = np.sqrt((trans**2).sum())
            bearing = np.arctan2(trans[0], trans[2]) * 180.0 / np.pi
            azimuth = np.arctan2(-trans[1], trans[2]) * 180.0 / np.pi
            detections.append({
                'id': r.tag_id,
                'center': r.center,
                'corners': r.corners,
                'distance': dist,
                'bearing': bearing,
                'azimuth': azimuth,
                'pose': pose[0],
                'pose_decomposed': self.decompose_pose_matrix(pose[0])
            })

            # print pose and pose_decomposed

            print("Pose:",detections[-1]['pose'])
            print("Pose Decomposed:",detections[-1]['pose_decomposed'])

            # Get the apriltag transform.
            transform = self.map_data['tags'][r.tag_id]['transform']
            print("Transform:",transform)

            current_detected_tags.add(r.tag_id)

            if r.tag_id == target:
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_distance", dist)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_bearing", bearing)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_azimuth", azimuth)
                self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/target_timestamp", frame_data.timestamp)

            tag_ids.append(r.tag_id)
            distances.append(dist)
            bearings.append(bearing)
            azimuths.append(azimuth)

        self.detected_tags = current_detected_tags
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/timestamp", frame_data.timestamp)
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/frame_id", frame_data.frame_id)
        self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/count", len(detections))
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/ids", tag_ids)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/distances", distances)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/bearings", bearings)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/azimuths", azimuths)

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

