import cv2
import numpy as np
import os
from typing import Dict, List, Tuple, Any
from sensors3140.maps.field_map import get_map

class LiveMapDisplay:
    def __init__(self, game_id: str):
        self.game_id = game_id
        self.image_path = os.path.join(os.path.dirname(__file__), 'images', f"{game_id}.jpg")
        self.img = None
        self.map_data = get_map(game_id)
        self.field_width, self.field_length = self.map_data.get_field_size()
        self.image_width = None
        self.image_height = None
        self.robot_width = 1.0
        self.robot_height = 1.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self._detections = {}

    def load(self) -> None:
        self.img = cv2.imread(self.image_path)
        assert self.img is not None, f"Could not load image {self.game_id}.jpg"
        self.image_width = self.img.shape[1]
        self.image_height = self.img.shape[0]

    def set_detected_tags(self, detections: List[Dict[str, Any]]) -> None:
        self._detections = {detection['id']: detection for detection in detections}

    def real_world_to_pixel(self, points: Tuple[float, float]) -> Tuple[int, int]:
        x, y = points
        y = self.field_width - y
        x = x / self.field_length * self.image_width
        y = y / self.field_width * self.image_height
        return int(x), int(y)

    def set_robot_size(self, width: float, height: float) -> None:
        self.robot_width = width / self.field_length * self.image_width
        self.robot_height = height / self.field_width * self.image_height

    def set_robot_position(self, x: float, y: float, rotation_degrees: float) -> None:
        self.robot_x = x
        self.robot_y = y
        self.robot_rotation_degrees = rotation_degrees

    def draw_robot(self) -> None:
        robot_color = (0, 255, 255)
        robot_outline_color = (0, 0, 0)
        robot_outline_thickness = 2
        points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]])
        points[:, 0] *= self.robot_width
        points[:, 1] *= self.robot_height
        angle = np.radians(self.robot_rotation_degrees)
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        points = np.dot(points, rotation_matrix.T) + np.array([self.robot_x, self.robot_y])
        new_points = [self.real_world_to_pixel((x, y)) for x, y in points]
        cv2.fillPoly(self.img, [np.array(new_points, np.int32)], robot_color)
        cv2.polylines(self.img, [np.array(new_points, np.int32)], True, robot_outline_color, robot_outline_thickness)
        x, y = self.real_world_to_pixel((self.robot_x, self.robot_y))
        cv2.circle(self.img, (x, y), 10, (0, 255, 0), -1)

    def display(self) -> None:
        img = self.img.copy()
        for tag_id in self.map_data.get_all_tags():
            x, y, z = self.map_data.get_tag_location(tag_id)
            pixel_x, pixel_y = self.real_world_to_pixel((x, y))
            color = (0, 0, 255)
            distance = None
            camera_pose = None
            if tag_id in self._detections:
                color = (0, 255, 0)
                distance = self._detections[tag_id]['distance']
                distance = distance / self.field_length * self.image_width
                camera_pose = self._detections[tag_id]['camera_pose']
            tag_transform = self.map_data.get_tag_transform(tag_id)
            tag_location = np.matmul(tag_transform, np.array([[0.0], [0.0], [0.0], [1.0]]))
            tag_direction = np.matmul(tag_transform, np.array([[0.0], [0.0], [-1.0], [1.0]]))
            if camera_pose is not None:
                camera_location = np.matmul(camera_pose, np.array([[0.0], [0.0], [0.0], [1.0]]))
                camera_direction = np.matmul(camera_pose, np.array([[0.0], [0.0], [1.0], [1.0]]))
                camera_location = np.matmul(tag_transform, camera_location)
                camera_direction = np.matmul(tag_transform, camera_direction)
                camera_trans = camera_location[:3] / camera_location[3]
                camera_dir = camera_direction[:3] / camera_direction[3]
                camera_color = (0, 255, 255)
                cam_x, cam_y = self.real_world_to_pixel((camera_trans[0], camera_trans[1]))
                cam_dir_x, cam_dir_y = self.real_world_to_pixel((camera_dir[0], camera_dir[1]))
                cv2.circle(img, (int(cam_x), int(cam_y)), 10, camera_color, -1)
                cv2.line(img, (int(cam_x), int(cam_y)), (int(cam_dir_x), int(cam_dir_y)), camera_color, 3)
                cv2.line(img, (int(pixel_x), int(pixel_y)), (int(cam_x), int(cam_y)), camera_color, 1)
            if distance is not None:
                cv2.circle(img, (int(pixel_x), int(pixel_y)), int(distance), (88, 88, 88), 2)
            tag_x, tag_y = self.real_world_to_pixel((tag_location[0], tag_location[1]))
            dir_x, dir_y = self.real_world_to_pixel((tag_direction[0], tag_direction[1]))
            cv2.line(img, (int(tag_x), int(tag_y)), (int(dir_x), int(dir_y)), color, 1)
            cv2.circle(img, (int(tag_x), int(tag_y)), 5, color, -1)
            text_size = cv2.getTextSize(str(tag_id), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = int(pixel_x - text_size[0] / 2)
            text_y = int(pixel_y - text_size[1] / 2)
            text_y += text_size[1] + 20 if text_y < self.image_height / 2 else -text_size[1] - 10
            cv2.putText(img, str(tag_id), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)
        window_name = 'AprilTag Overlay'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, img)

if __name__ == "__main__":
    display = LiveMapDisplay("2025-reefscape")
    display.load()
    display.set_robot_size(0.5, 0.5)
    display.set_robot_position(0.0, 0.0, 30)
    display.display()
    cv2.waitKey(0)
    cv2.destroyAllWindows()