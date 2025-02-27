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
        self.decision_quality_average = None

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
        hamming = []
        goodness = []
        decision_margin = []

        target = self.table.getInteger(f"sensors3140/apriltags/camera{self.camera_id}/target_id")

        best_camera_pose = None
        best_camera_translation = None
        best_camera_direction = None
        best_camera_tag_id = None
        for r in results:
            # camera param format: [fx, fy, cx, cy]
            
            pose = self.detector.detection_pose(r, self.camera_params, self.tag_size)
            trans = pose[0][0:3, 3]
            dist = np.sqrt((trans**2).sum())
            print(f"Tag {r.tag_id} location: {trans.flatten()} distance: {dist}")
            bearing = np.arctan2(trans[0], trans[2]) * 180.0 / np.pi
            azimuth = np.arctan2(-trans[1], trans[2]) * 180.0 / np.pi
            goodness.append(r.goodness)
            hamming.append(r.hamming)
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

            # This transforms points in the tags coordinate system to the field coordinate system
            #april_tag_pose = self.map_data['tags'][r.tag_id]['tag_transform']

            #print(f"Tag {r.tag_id} pose: {april_tag_pose}")

            # this is the pose of the tag in the camera coordinate system
            tag_pose = pose[0]
            #print(f"Tag {r.tag_id} camera pose: {tag_pose}")

            detections[-1]['tag_pose'] = tag_pose

            # find the camera pose in the tag coordinate system
            camera_pose = np.linalg.inv(tag_pose)

            detections[-1]['camera_pose'] = camera_pose

            camera_translation = np.dot(camera_pose, np.array([[0], [0], [0], [1]]))
            camera_translation = camera_translation[:3]/camera_translation[3]

            #print(f"Tag {r.tag_id} camera translation: {camera_translation}")
            if r.tag_id >= len(self.map_data['tags']):
                continue

            tag_transform = self.map_data['tags'][r.tag_id]['tag_transform']

            camera_location = np.array([[0.0], [0.0], [0.0], [1.0]]) # in homogenous coordinates
            camera_direction = np.array([[0.0], [0.0], [1.0], [1.0]]) # in homogenous coordinates

            camera_location = np.matmul(camera_pose, camera_location)
            camera_direction = np.matmul(camera_pose, camera_direction)

            camera_location = np.matmul(tag_transform, camera_location)
            camera_direction = np.matmul(tag_transform, camera_direction)
            camera_trans = camera_location[:3] / camera_location[3]
            camera_dir = camera_direction[:3] / camera_direction[3]

            # Score should be related to how close the camera is to the tag and how close the camera is to the axis of the tag.
            # The closer the camera is to the tag, the higher the score.
            # The score should be the highest when the camera is 45 degrees to the axis of the tag.
            camera_location_angle = np.arctan2(camera_pose[1,3], camera_pose[0,3])
            score = camera_location_angle

            detections[-1]['camera_location'] = camera_location
            detections[-1]['camera_direction'] = camera_direction
            detections[-1]['camera_translation'] = camera_trans
            detections[-1]['camera_location_score'] = score
            detections[-1]['camera_pose'] = camera_pose

            if best_camera_pose is None or score > best_camera_score:
                best_camera_pose = camera_pose
                best_camera_direction = camera_dir
                best_camera_tag_id = r.tag_id
                best_camera_translation = camera_trans
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
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/goodness", goodness)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/hamming", hamming)
        self.table.setDoubleArray(f"sensors3140/apriltags/camera{self.camera_id}/decision_margin", decision_margin)
        # compute average hamming distance
        if len(hamming) > 0:
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_hamming", sum(hamming) / len(hamming))
        else:
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_hamming", 0)

        # compute average goodness
        if len(goodness) > 0:
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_goodness", sum(goodness) / len(goodness))
        else:
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_goodness", 0)

        # compute average decision margin
        if len(decision_margin) > 0:
            avereage_decision_margin = sum(decision_margin) / len(decision_margin)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/average_decision_margin", avereage_decision_margin)
            decision_max = 80.0
            decision_min = 65.0
            decision_quality = (sum(decision_margin) / len(decision_margin) - decision_min) / (decision_max - decision_min)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/decision_quality_current", decision_quality)
            if self.decision_quality_average is None:
                self.decision_quality_average = decision_quality
            else:
                self.decision_quality_average = (self.decision_quality_average * 0.98) + (decision_quality * 0.02)
            self.table.setDouble(f"sensors3140/apriltags/camera{self.camera_id}/decision_quality_average", self.decision_quality_average)
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

