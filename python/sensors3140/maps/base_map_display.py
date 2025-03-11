import cv2
import numpy as np
import os
from typing import Dict, List, Tuple, Any
from sensors3140.maps.field_map import get_map

class BaseMapDisplay:
    """
    Base class for displaying a field map with robot and camera positions.
    """
    def __init__(self, game_id: str):
        """
        Initialize the BaseMapDisplay
        
        Args:
            game_id: ID of the game field map to use
        """
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

        self.camera_positions = {}  # camera_id -> (x, y, z, rotation, tag_id, tag_distance, elapsed_time)

    def load(self) -> None:
        """
        Load the field map image
        """
        self.img = cv2.imread(self.image_path)
        assert self.img is not None, f"Could not load image {self.game_id}.jpg"
        self.image_width = self.img.shape[1]
        self.image_height = self.img.shape[0]

    def update_camera_position(self, camera_id: int, camera_xyz: Tuple[float, float, float], camera_angle: float, tag_id: int = None, tag_distance: float = None, elapsed_time: float = None) -> None:
        """
        Update the position of a camera
        
        Args:
            camera_id: ID of the camera
            camera_xyz: (x, y, z) position of the camera
            camera_angle: Rotation angle of the camera
            tag_id: ID of the tag being detected
            tag_distance: Distance to the detected tag
            elapsed_time: Time elapsed since the last update
        """
        if len(camera_xyz) >= 3:
            self.camera_positions[camera_id] = (camera_xyz[0], camera_xyz[1], camera_xyz[2], camera_angle, tag_id, tag_distance, elapsed_time)
        else:
            raise ValueError(f"camera_xyz must have at least 3 elements, got {len(camera_xyz)}")

    def set_detected_tags(self, detections: List[Dict[str, Any]]) -> None:
        """
        Set the detected tags
        
        Args:
            detections: List of detected tags
        """
        self._detections = {detection['id']: detection for detection in detections}

    def real_world_to_pixel(self, points: Tuple[float, float]) -> Tuple[int, int]:
        """
        Convert real-world coordinates to pixel coordinates
        
        Args:
            points: (x, y) coordinates in the real world
        
        Returns:
            (x, y) coordinates in pixels
        """
        x, y = points
        y = self.field_width - y
        x = x / self.field_length * self.image_width
        y = y / self.field_width * self.image_height
        return int(x), int(y)

    def set_robot_size(self, width: float, height: float) -> None:
        """
        Set the size of the robot
        
        Args:
            width: Width of the robot in meters
            height: Height of the robot in meters
        """
        self.robot_width = width / self.field_length * self.image_width
        self.robot_height = height / self.field_width * self.image_height

    def set_robot_position(self, x: float, y: float, rotation_degrees: float) -> None:
        """
        Set the position and rotation of the robot
        
        Args:
            x: X-coordinate of the robot
            y: Y-coordinate of the robot
            rotation_degrees: Rotation angle of the robot in degrees
        """
        self.robot_x = x
        self.robot_y = y
        self.robot_rotation_degrees = rotation_degrees

    def draw_robot(self) -> None:
        """
        Draw the robot on the field map
        """
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

    def draw_camera_positions(self, img) -> None:
        """
        Draw the positions of the cameras on the field map
        
        Args:
            img: Image to draw on
        """
        for camera_id, (x, y, z, rotation, tag_id, tag_distance, elapsed_time) in self.camera_positions.items():
            if elapsed_time > 2:
                continue
            camera_color = (0, 0, 0)
            
            # if the time is less than .25 use full color and then color fades over the next 2 seconds
            if elapsed_time is not None:
                if elapsed_time < 0.25:
                    camera_color = (0, 255, 255)
                else:
                    fade = 1 - (elapsed_time - 0.25) / 2
                    camera_color = (int(0 * fade), int(255 * fade), int(255 * fade))

            # minimum color is 0,0,0
            camera_color = (max(0, camera_color[0]), max(0, camera_color[1]), max(0, camera_color[2]))
            tag_color = (0, 255, 0)
            pixel_x, pixel_y = self.real_world_to_pixel((x, y))

            tag_x, tag_y, tag_z = self.map_data.get_tag_location(tag_id)
            tag_x, tag_y = self.real_world_to_pixel((tag_x, tag_y))
            
            if tag_id is not None:
                # draw a line to the tag location
                cv2.line(img, (pixel_x, pixel_y), (tag_x, tag_y), tag_color, 3)
                # draw a circle of radius tag_distance
                if tag_distance is not None:
                    cv2.circle(img, (tag_x, tag_y), int(tag_distance / self.field_length * self.image_width), tag_color, 2)
                    # print the tag distance at the tag location
                    cv2.putText(img, f"{tag_distance:.2f}m", (tag_x, tag_y), cv2.FONT_HERSHEY_SIMPLEX, 1, tag_color, 2)
            # draw the camera location
            cv2.circle(img, (pixel_x, pixel_y), 15, camera_color, -1)
            cv2.putText(img, f"{camera_id}", (pixel_x, pixel_y), cv2.FONT_HERSHEY_SIMPLEX, 1, camera_color, 2)

    def display(self,best_camera_location=None, best_camera_direction=None) -> None:
        """
        Display the field map with the robot and camera positions
        """
        img = self.img.copy()

        self.draw_camera_positions(img)

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
            tag_left = np.matmul(tag_transform, np.array([[-0.165], [0.0], [0.0], [1.0]]))
            tag_right = np.matmul(tag_transform, np.array([[0.165], [0.0], [0.0], [1.0]]))
                                 
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

            # write the tag location to the csv file

            tag_left_x, tag_left_y = self.real_world_to_pixel((tag_left[0], tag_left[1]))
            tag_right_x, tag_right_y = self.real_world_to_pixel((tag_right[0], tag_right[1]))
            #purple
            reef_color = (255, 0, 255)
            cv2.circle(img, (int(tag_left_x), int(tag_left_y)), 5, reef_color, -1)
            cv2.circle(img, (int(tag_right_x), int(tag_right_y)), 5, reef_color, -1)
            text_size = cv2.getTextSize(str(tag_id), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = int(pixel_x - text_size[0] / 2)
            text_y = int(pixel_y - text_size[1] / 2)
            text_y += text_size[1] + 20 if text_y < self.image_height / 2 else -text_size[1] - 10
            cv2.putText(img, str(tag_id), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

        if best_camera_location is not None and best_camera_direction is not None:
            cam_x, cam_y = self.real_world_to_pixel((best_camera_location[0], best_camera_location[1]))
            cam_dir_x, cam_dir_y = self.real_world_to_pixel((best_camera_direction[0], best_camera_direction[1]))
            cv2.circle(img, (int(cam_x), int(cam_y)), 10, (255, 0, 255), -1)
            cv2.line(img, (int(cam_x), int(cam_y)), (int(cam_dir_x), int(cam_dir_y)), (255, 0, 255), 3)

        window_name = 'AprilTag Overlay'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, img)