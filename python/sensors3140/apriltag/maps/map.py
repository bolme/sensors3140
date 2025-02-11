# Use opencv to load in 2025_reefscape.jpg and 2025_reefscape.json and display the image with april tags overlayed on top of the image.
# This supports the FRC 2025 game Reefscape. The april tags are used to determine the position of the robot on the field.

import cv2
import numpy as np
import json
import argparse
import os

# References:
#
# Robot Coordinate System: https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html

def load_apriltag_coordinates(game_id):
    coordinates = {}
    with open(f"{game_id}.json") as f:
        data = json.load(f)
        for tag in data['tags']:
            id = tag['ID']
            x = tag['pose']['translation']['x']
            y = tag['pose']['translation']['y']
            z = tag['pose']['translation']['z']
            
            qw = tag['rotation']['quaternion']['W']
            qx = tag['rotation']['quaternion']['X']
            qy = tag['rotation']['quaternion']['Y']
            qz = tag['rotation']['quaternion']['Z']

            # Create a transform that converts from the tag coordinate system to the field coordinate system
            
            rotation_matrix = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                                        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                                        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])
            translation_matrix = np.array([[x], [y], [z]])

            transform_matrix = np.eye(4)
            transform_matrix[:3, :3] = rotation_matrix
            transform_matrix[:3, 3] = translation_matrix.flatten()

            coordinates[id] = ((x, y, z), transform_matrix)


class LiveMapDisplay:
    def __init__(self, game_id):
        self.game_id = game_id
        self.image_path = os.path.join(os.path.dirname(__file__), f"{game_id}.jpg")
        self.json_path = os.path.join(os.path.dirname(__file__), f"{game_id}.json")
        self.img = None
        self.data = None
        self.field_width = None
        self.field_length = None
        self.image_width = None
        self.image_height = None

    def load(self):
        self.img = cv2.imread(self.image_path)
        assert self.img is not None, f"Could not load image {self.game_id}.jpg"

        with open(self.json_path) as f:
            self.data = json.load(f)

        self.field_width = self.data['field']['width']
        self.field_length = self.data['field']['length']
        self.image_width = self.img.shape[1]
        self.image_height = self.img.shape[0]

    def real_world_to_pixel(self, points):
        x = points[0]
        y = points[1]

        y = self.field_width - y
        x = x / self.field_length * self.image_width
        y = y / self.field_width * self.image_height

        return int(x), int(y)
    
    def set_robot_size(self, width, height):
        # convert to pixels
        width = width / self.field_length * self.image_width
        height = height / self.field_width * self.image_height
        self.robot_width = width
        self.robot_height = height
    
    def set_robot_position(self, x, y, rotation_degrees):
        # convert to pixels
        self.robot_x = x
        self.robot_y = y
        self.robot_rotation_degrees = rotation_degrees

    def draw_robot(self):
        # draw the robot
        # draw the robot
        robot_color = (0, 255, 255)
        robot_outline_color = (0, 0, 0)
        robot_outline_thickness = 2

        # Computer the robot corner points
        points = np.array([[0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]])
        # Rescale the points to the robot size
        points[:, 0] *= self.robot_width
        points[:, 1] *= self.robot_height

        # Compute a rotation and translation matrix
        angle = np.radians(self.robot_rotation_degrees)
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        translation_matrix = np.array([self.robot_x, self.robot_y])
        points = np.dot(points, rotation_matrix.T) + translation_matrix

        # convert to pixel coordinates
        new_points = [self.real_world_to_pixel((x, y)) for x, y in points]

        # Draw the robot
        cv2.fillPoly(self.img, [np.array(new_points, np.int32)], robot_color)
        cv2.polylines(self.img, [np.array(new_points, np.int32)], True, robot_outline_color, robot_outline_thickness)

        # Draw the robot center point
        x, y = self.real_world_to_pixel((self.robot_x, self.robot_y))
        print(x, y) 
        cv2.circle(self.img, (x, y), 10, (0, 255, 0), -1)


        
    def display(self):
        # Create a 3x3 transform that converts real world coordinates to pixel coordinates
        #rotation_matrix = np.eye(3)
        #scale_matrix = np.array([[self.image_width / self.field_width, 0, 0], [0, self.image_height / self.field_length, 0], [0, 0, 1]])
        #translation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        #transform_matrix = np.dot(scale_matrix, rotation_matrix)
        #transform_matrix = np.dot(transform_matrix,translation_matrix)
    
        for tag in self.data['tags']:
            id = tag['ID']
            x = tag['pose']['translation']['x']
            y = tag['pose']['translation']['y']
            z = tag['pose']['translation']['z']

            tag_point = np.array([[x], [y], [1.0]])

            pixel_x, pixel_y = self.real_world_to_pixel((x, y))

            cv2.circle(self.img, (int(pixel_x),int(pixel_y)), 10, (0, 0, 255), -1)

            text_size = cv2.getTextSize(str(id), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = int(pixel_x - text_size[0] / 2)
            text_y = int(pixel_y - text_size[1] / 2)
            if text_y < self.image_height / 2:
                text_y += text_size[1] + 20
            else:
                text_y -= text_size[1] - 10

            cv2.putText(self.img, str(id), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        # plot a circle at the robot's location
        robot_x, robot_y = self.real_world_to_pixel((self.robot_x, self.robot_y))
        print(robot_x, robot_y)
        print(self.robot_x, self.robot_y)
        cv2.circle(self.img, (robot_x, robot_y), 10, (0, 255, 0), 10)


        window_name = 'AprilTag Overlay'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, self.img)
        cv2.waitKey(30000)


if __name__ == "__main__":
    display = LiveMapDisplay("2025-reefscape")
    display.load()
    display.set_robot_size(0.5, 0.5)
    display.set_robot_position(0.0, 0.0, 30)
    display.display()


    
