# a live map that displays infor from network table.

# Use opencv to load in 2025_reefscape.jpg and 2025_reefscape.json and display the image with april tags overlayed on top of the image.
# This supports the FRC 2025 game Reefscape. The april tags are used to determine the position of the robot on the field.
print( 'NTMapStart')

import cv2
import numpy as np
import json
import argparse
import os
print("Loading networktables")
import sensors3140.tables.network_tables as nt

print('NTMap Load.')

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


class NTMapDisplay:
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

        self.timestamp = -1.0

        self.tables = nt.NetworkTablesManager()


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

    def set_current_detections(self,tag_ids, distances, bearings):
        try:
            assert len(tag_ids) == len(distances) == len(bearings), "Tag, distance, and bearing arrays must be the same length"
            self.tag_ids = tag_ids
            self.distances = distances
            self.bearings = bearings
        except:
            print("Warning: Could not set AprilTag Detections")

    def set_camera_pose(self, best_camera_translation, best_camera_direction, best_camera_pose_tag):
        self.best_camera_translation = best_camera_translation
        self.best_camera_direction = best_camera_direction
        self.best_camera_pose_tag = best_camera_pose_tag
    

    def update(self):
        '''Update from Network Tables'''
        print("Connected to Network Tables:",self.tables.is_connected())
        # Get the timestamp
        print("Timestamp:",self.tables.getDouble("sensors3140/timestamp"))

        tag_ids = self.tables.getIntegerArray("sensors3140/apriltags/camera1/ids",[])
        distances = self.tables.getDoubleArray("sensors3140/apriltags/camera1/distances",[])
        bearings = self.tables.getDoubleArray("sensors3140/apriltags/camera1/bearings",[])
        self.set_current_detections(tag_ids, distances, bearings)

        # update camera location data from network tables - best_camera_translation and best_camera_direction
        best_camera_translation = self.tables.getDoubleArray("sensors3140/apriltags/camera1/camera_position")
        best_camera_direction = self.tables.getDoubleArray("sensors3140/apriltags/camera1/camera_direction")
        best_camera_pose_tag = self.tables.getInteger("sensors3140/apriltags/camera1/camera_position_tag",-1)

        self.set_camera_pose(best_camera_translation, best_camera_direction, best_camera_pose_tag)

        print("Translation:",best_camera_translation)



        
    def display(self):
        """Display the field map with AprilTags and detected positions."""
        # Create a copy of the image to draw on
        display_img = self.img.copy()
        
        # Draw each tag on the field
        for tag in self.map_data['tags']:
            tag_id = tag['ID']
            tag_x = tag['pose']['translation']['x']
            tag_y = tag['pose']['translation']['y']
            tag_z = tag['pose']['translation']['z']
            
            # Calculate pixel coordinates for this tag
            pixel_x, pixel_y = self.real_world_to_pixel((tag_x, tag_y))
            
            # Default color for tags (red)
            tag_color = (0, 0, 255)
            tag_distance = None
            
            # Check if this tag is currently being detected
            if tag_id in self.tag_ids:
                # Get distance for this tag and change color to green for detected tags
                tag_distance = dict(zip(self.tag_ids, self.distances))[tag_id]
                tag_color = (0, 255, 0)
                # Convert distance to pixel radius
                tag_distance = tag_distance / self.field_length * self.image_width
                
                # If this is the tag used for camera pose estimation
                if self.best_camera_pose_tag == tag_id:
                    # Draw camera position and direction
                    cam_x, cam_y = self.real_world_to_pixel((self.best_camera_translation[0], self.best_camera_translation[1]))
                    cam_dir_x, cam_dir_y = self.real_world_to_pixel((self.best_camera_direction[0], self.best_camera_direction[1]))
                    
                    # Yellow dot for camera position
                    cv2.circle(display_img, (int(cam_x), int(cam_y)), 10, (0, 255, 255), -1)
                    # Line showing camera direction
                    cv2.line(display_img, (int(cam_x), int(cam_y)), (int(cam_dir_x), int(cam_dir_y)), (0, 255, 255), 3) 
                    # Line connecting tag to camera
                    cv2.line(display_img, (int(pixel_x), int(pixel_y)), (int(cam_x), int(cam_y)), (0, 255, 255), 1)
            
            # Calculate tag orientation using the tag transform matrix
            tag_transform = tag['tag_transform']
            
            # Calculate tag position in world coordinates
            tag_location = np.array([[0.0], [0.0], [0.0], [1.0]])  # Homogeneous coordinates 
            tag_location = np.matmul(tag_transform, tag_location)
            
            # Calculate tag direction vector (1m in front of tag)
            tag_direction = np.array([[0.0], [0.0], [-1.0], [1.0]])  # Homogeneous coordinates
            tag_direction = np.matmul(tag_transform, tag_direction)
            
            # Draw distance circle if tag is detected
            if tag_distance is not None:
                cv2.circle(display_img, (int(pixel_x), int(pixel_y)), int(tag_distance), (88, 88, 88), 2)
            
            # Draw tag direction indicator
            tag_pixel_x, tag_pixel_y = self.real_world_to_pixel((tag_location[0], tag_location[1]))
            dir_pixel_x, dir_pixel_y = self.real_world_to_pixel((tag_direction[0], tag_direction[1]))
            cv2.line(display_img, (int(tag_pixel_x), int(tag_pixel_y)), 
                    (int(dir_pixel_x), int(dir_pixel_y)), tag_color, 1)
            cv2.circle(display_img, (int(tag_pixel_x), int(tag_pixel_y)), 5, tag_color, -1)
            
            # Draw tag ID label
            text_size = cv2.getTextSize(str(tag_id), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            text_x = int(pixel_x - text_size[0] / 2)
            text_y = int(pixel_y - text_size[1] / 2)
            
            # Position text above or below the tag depending on tag's position on field
            if text_y < self.image_height / 2:
                text_y += text_size[1] + 20
            else:
                text_y -= text_size[1] - 10
                
            cv2.putText(display_img, str(tag_id), (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, tag_color, 2, cv2.LINE_AA)

        # Display the final image
        window_name = 'AprilTag Overlay'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, display_img)

def parse_arguments():
    parser = argparse.ArgumentParser(description='AprilTag Field Map Display')
    parser.add_argument('--server', default='127.0.0.1', help='NetworkTables server address')
    parser.add_argument('--game', default='2025-reefscape', help='Game ID for field map')
    return parser.parse_args()

if __name__ == "__main__":
    print('Creating Map...')
    args = parse_arguments()
    
    # Initialize network tables with the server address
    nt.NetworkTablesManager(args.server)
    
    # Create the display with the specified game ID
    display = NTMapDisplay(args.game)
    display.load()
    print(f'Loaded {args.game} map...')
    print(f'Connected to NetworkTables server: {args.server}')
    
    while True:
        display.update()
        display.display()
        key = cv2.waitKey(1)
        # Check for q to quit
        if key == ord('q'):
            break

    print("Closing Map")
    cv2.destroyAllWindows()
    
    
    
