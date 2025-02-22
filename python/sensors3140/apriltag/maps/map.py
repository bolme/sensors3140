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


# Field coordinate notes:
# The x-axis is the long axis of the field, From the blue side to the red side
# The z-axis is the vertical axis, from the floor to the ceiling
# The y-axis is the short axis of the field in right-hand rule.  It runs to the left of the blue side to the right of the red side

# Tag coordinate notes:
# When detected by the apriltag library it is in image coordinates.  If the tag is upright:
# The z-axes is depth and is positive away from the camera
# The x-axis is positive to the right
# The y-axis is positive down

def load_apriltags(game_id):
    json_path = os.path.join(os.path.dirname(__file__), f"{game_id}.json")
    with open(json_path) as f:
        data = json.load(f)
        for tag in data['tags']:
            id = tag['ID']
            x = tag['pose']['translation']['x']
            y = tag['pose']['translation']['y']
            z = tag['pose']['translation']['z']

            qw = tag['pose']['rotation']['quaternion']['W']
            qx = tag['pose']['rotation']['quaternion']['X']
            qy = tag['pose']['rotation']['quaternion']['Y']
            qz = tag['pose']['rotation']['quaternion']['Z']


            # Create a transform that converts from the tag coordinate system to the field coordinate system
            rotation_matrix = np.array([[1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
                                        [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
                                        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]])
            
            translation_matrix = np.array([[x], [y], [z]])

            field_transform = np.eye(4)
            field_transform[:3, :3] = rotation_matrix
            field_transform[:3, 3] = translation_matrix.flatten()
            
            # add the transform to the dictionary
            tag['field_transform'] = field_transform

            # we need a rotation matrix that converts from the field coordinate system to the tag coordinate system
            
            # Rewriting the field coordinate system to the tag coordinate system
            # z => -x
            # x => y
            # y => -z
            correction = np.zeros((4, 4))
            correction[:3,:3] = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]],dtype=np.float32)
            correction[3,3] = 1

            tag_transform = np.dot(field_transform, correction)

            tag['tag_transform'] = tag_transform

        return data            

    return None


class LiveMapDisplay:
    def __init__(self, game_id):
        self.game_id = game_id
        self.image_path = os.path.join(os.path.dirname(__file__), f"{game_id}.jpg")
        self.img = None

        self.map_data = load_apriltags(game_id)

        self.field_width = None
        self.field_length = None
        self.image_width = None
        self.image_height = None

        self.robot_width = 1.0
        self.robot_height = 1.0
        self.robot_x = 0.0
        self.robot_y = 0.0

        self._detections = {}

    def load(self):
        self.img = cv2.imread(self.image_path)
        assert self.img is not None, f"Could not load image {self.game_id}.jpg"

        self.field_width = self.map_data['field']['width']
        self.field_length = self.map_data['field']['length']
        self.image_width = self.img.shape[1]
        self.image_height = self.img.shape[0]

    def set_detected_tags(self, detections):
        self._detections = {}
        for detection in detections:
            id = detection['id']
            self._detections[id] = detection


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

        cv2.circle(self.img, (x, y), 10, (0, 255, 0), -1)


        
    def display(self):
        # Create a 3x3 transform that converts real world coordinates to pixel coordinates
        #rotation_matrix = np.eye(3)
        #scale_matrix = np.array([[self.image_width / self.field_width, 0, 0], [0, self.image_height / self.field_length, 0], [0, 0, 1]])
        #translation_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        #transform_matrix = np.dot(scale_matrix, rotation_matrix)
        #transform_matrix = np.dot(transform_matrix,translation_matrix)

        img = self.img.copy()
    
        for tag in self.map_data['tags']:
            id = tag['ID']
            x = tag['pose']['translation']['x']
            y = tag['pose']['translation']['y']
            z = tag['pose']['translation']['z']

                                           
            # Compute the pixel coordinates of the tag
            tag_point = np.array([[x], [y], [1.0]])
            pixel_x, pixel_y = self.real_world_to_pixel((x, y))

            color = (0, 0, 255)
            distance = None

            camera_pose = None
            if id in self._detections:
                color = (0, 255, 0)
                distance = self._detections[id]['distance']
                # convert the distace to a radius in pixels
                distance = distance / self.field_length * self.image_width
                camera_pose = self._detections[id]['camera_pose']

            if False:
                # Draw the tag
                cv2.circle(img, (int(pixel_x),int(pixel_y)), 10, color, -1)

                # Draw the tag real world coordinates
                text = f"({x:.2f}, {y:.2f}" #, {z:.2f})"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                text_x = int(pixel_x - text_size[0] / 2)
                text_y = int(pixel_y + text_size[1] + 10)
                cv2.putText(img, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)



            # Find the point one meter in front of the tag so we can draw a line to it
            # This is the x axis of the tag coordinate system
            tag_transform = tag['tag_transform']

            tag_location = np.array([[0.0], [0.0], [0.0], [1.0]]) # in homogenous coordinates
            tag_location = np.matmul(tag_transform, tag_location)

            tag_direction = np.array([[0.0], [0.0], [-1.0], [1.0]]) # in homogenous coordinates
            tag_direction = np.matmul(tag_transform, tag_direction)

            if camera_pose is not None:
                camera_location = np.array([[0.0], [0.0], [0.0], [1.0]]) # in homogenous coordinates
                camera_direction = np.array([[0.0], [0.0], [1.0], [1.0]]) # in homogenous coordinates

                camera_location = np.matmul(camera_pose, camera_location)
                camera_direction = np.matmul(camera_pose, camera_direction)

                camera_location = np.matmul(tag_transform, camera_location)
                camera_direction = np.matmul(tag_transform, camera_direction)
                camera_trans = camera_location[:3] / camera_location[3]
                camera_dir = camera_direction[:3] / camera_direction[3]

                #yellow
                camera_color = (0, 255, 255)
                cam_x, cam_y = self.real_world_to_pixel((camera_trans[0], camera_trans[1]))
                cam_dir_x, cam_dir_y = self.real_world_to_pixel((camera_dir[0], camera_dir[1]))
                cv2.circle(img, (int(cam_x),int(cam_y)), 10, camera_color, -1)
                cv2.line(img, (int(cam_x), int(cam_y)), (int(cam_dir_x), int(cam_dir_y)), camera_color, 3)

                # draw a  line from the tag to the camera
                cv2.line(img, (int(pixel_x), int(pixel_y)), (int(cam_x), int(cam_y)), camera_color, 1)

            # Draw the distance
            if distance is not None:
                cv2.circle(img, (int(pixel_x),int(pixel_y)), int(distance), (88,88,88), 2)
            
            # Draw a line from the tag to the point one meter in front of the tag
            tag_x, tag_y = self.real_world_to_pixel((tag_location[0], tag_location[1]))
            dir_x, dir_y = self.real_world_to_pixel((tag_direction[0], tag_direction[1]))
            cv2.line(img, (int(tag_x), int(tag_y)), (int(dir_x), int(dir_y)), color, 1)
            cv2.circle(img, (int(tag_x),int(tag_y)), 5, color, -1)




            text_size = cv2.getTextSize(str(id), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = int(pixel_x - text_size[0] / 2)
            text_y = int(pixel_y - text_size[1] / 2)
            if text_y < self.image_height / 2:
                text_y += text_size[1] + 20
            else:
                text_y -= text_size[1] - 10

            cv2.putText(img, str(id), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2, cv2.LINE_AA)

            #if global_camera_pose is not None:
            #    # Draw a line from the tag to the camera
            #    blue = (77, 255, 255)
            #
            #    print(f"Global Tag Location: {tag_point.flatten()}")

            #    camera_x, camera_y = self.real_world_to_pixel((global_camera_pose[0,3], global_camera_pose[1,3]))
            #    cv2.line(img, (int(pixel_x), int(pixel_y)), (int(camera_x), int(camera_y)), blue, 1)
            #    cv2.circle(img, (int(camera_x),int(camera_y)), 10, blue, -1)


        # plot a circle at the robot's location
        #robot_x, robot_y = self.real_world_to_pixel((self.robot_x, self.robot_y))
        #cv2.circle(img, (robot_x, robot_y), 10, (0, 255, 0), 10)


        window_name = 'AprilTag Overlay'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, img)

if __name__ == "__main__":
    display = LiveMapDisplay("2025-reefscape")
    display.load()
    display.set_robot_size(0.5, 0.5)
    display.set_robot_position(0.0, 0.0, 30)
    display.display()


    
