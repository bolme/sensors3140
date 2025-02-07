import cv2
import apriltag
import numpy as np

class AprilTagDetector:
    def __init__(self, tag_family, camera_params, tag_size):
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

    def __call__(self, frame, camera_params=None, tag_size=None):
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
        for r in results:
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
                'pose': pose[0]
            })
        return detections

