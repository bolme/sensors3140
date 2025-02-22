import sensors3140 as sensors3140
import os
import os.path
import json
import argparse
import cv2
from sensors3140.apriltag.detector import AprilTagDetector
from sensors3140.camera.streaming_task import StreamingTask
import numpy as np

import sensors3140.tables.network_tables as nt

import sensors3140.apriltag.maps.map as map

import time

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sensors3140")

    parser.add_argument("--config", type=str, default="config.json", help="Path to the configuration file")
    parser.add_argument("--display" , action="store_true", help="Display the camera feed", default=False)
    parser.add_argument("--map", action="store_true", help="Display the map", default=False)
    return parser.parse_args()

def display_apriltag_boxes(img, detections):
    for detection in detections:
        # Draw the tag outline
        corners = detection['corners']
        for i in range(len(corners)):
            cv2.line(img, tuple(corners[i-1, :].astype(int)), tuple(corners[i, :].astype(int)), (0, 255, 0), 2)

        # Draw the center
        center = detection['center']
        cv2.circle(img, tuple(center.astype(int)), 5, (0, 255, 0), -1)

        # Draw the ID and distance
        cv2.putText(img, f"ID: {detection['id']}", tuple(center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, f"Dist: {detection['distance']:.2f}m", (center[0].astype(int), center[1].astype(int) + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    return img


def display_apriltag_pose(img, detections):
    for detection in detections:
        pose = detection['pose']

        tag_translation = pose[0:3, 3]
        tag_rotation = pose[0:3, 0:3]

        camera_params = detection['camera_params']

        # tag center is the translation in homogenous coordinates
        tag_center = np.array([[tag_translation[0]], [tag_translation[1]], [tag_translation[2]], [1.0]])

        # Compute the 3x4 camera projection matrix
        kx, ky, cx, cy = camera_params

        camera_matrix = np.array([[kx, 0, cx], [0, ky, cy], [0, 0, 1]])
        transform = np.eye(4)
        projection_matrix = np.dot(camera_matrix, transform[:3, :])
        #print("Proj:",projection_matrix.shape)
        #print(projection_matrix)

        transformed_center = np.dot(projection_matrix, tag_center)
        #print("Center:",transformed_center)

        image_coords = transformed_center[0:2] / transformed_center[2]
        #print("Image Coords:",image_coords)

        # Draw the tag_center as a yellow circle outline
        cv2.circle(img, tuple(image_coords[0:2].flatten().astype(int)), 30, (0, 255, 255), 2)

        # Draw 0.1m x, y, z axes for the tag
        x_axis = np.array([[0.1], [0], [0], [1]])
        y_axis = np.array([[0], [0.1], [0], [1]])
        z_axis = np.array([[0], [0], [-0.1], [1]])

        x_axis = np.dot(pose, x_axis)
        y_axis = np.dot(pose, y_axis)
        z_axis = np.dot(pose, z_axis)

        x_axis = np.dot(projection_matrix, x_axis) 
        y_axis = np.dot(projection_matrix, y_axis)
        z_axis = np.dot(projection_matrix, z_axis) 

        # Normalize the coordinates to image space
        x_axis = x_axis[0:2] / x_axis[2]
        y_axis = y_axis[0:2] / y_axis[2]
        z_axis = z_axis[0:2] / z_axis[2]

        # Draw the axes
        cv2.line(img, tuple(image_coords[0:2].flatten().astype(int)), tuple(x_axis[0:2].flatten().astype(int)), (0, 0, 255), 2)
        cv2.line(img, tuple(image_coords[0:2].flatten().astype(int)), tuple(y_axis[0:2].flatten().astype(int)), (0, 255, 0), 2)
        cv2.line(img, tuple(image_coords[0:2].flatten().astype(int)), tuple(z_axis[0:2].flatten().astype(int)), (255, 0, 0), 2)

        # Label x, y, z axes
        cv2.putText(img, "X", tuple(x_axis[0:2].flatten().astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(img, "Y", tuple(y_axis[0:2].flatten().astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(img, "neg Z", tuple(z_axis[0:2].flatten().astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)              

    return img




def main():

    map_display = None

    # Parse command line arguments
    args = parse_args()

    files = os.listdir(sensors3140.sensors3140_directory)

    # Scan the configuration directory for network configuration
    network_config_path = os.path.join(sensors3140.sensors3140_directory, "network.json")
    if not os.path.exists(network_config_path):
        print(f"Network configuration not found.  Creating at {network_config_path}")
        data = {
            "network_table_ip": "10.31.40.2",
            "network_table_port": 1735
        }
        with open(network_config_path, "w") as f:
            json.dump(data, f, indent=4)

    with open(os.path.join(sensors3140.sensors3140_directory, "network.json"), "r") as f:
        data = json.load(f)
        print("Loaded network configuration")


    tables = nt.NetworkTablesManager(data["network_table_ip"])
    # wait for network tables to connect
    for _ in range(50): 
        if tables.is_connected():
            break
        time.sleep(0.1)
    if not tables.is_connected():
        print("WARNING: Failed to connect to NetworkTables")
    else:
        print("Connected to NetworkTables")


    # Scan the configuration directory for camera configurations
    cameras = []
    for file in files:
        if file.startswith("camera_") and file.endswith(".json"):
            print(file)
            data = {}
            with open(os.path.join(sensors3140.sensors3140_directory, file), "r") as f:
                data = json.load(f)
            print(data)

            # Create a camera object from the configuration
            camera = sensors3140.Camera(**data)

            # Add the camera to the list
            cameras.append(camera)

            print("Created camera", camera.camera_id)

    print(f"Found {len(cameras)} cameras in directory {sensors3140.sensors3140_directory}.")

    time.sleep(3)

    at_detectors = [AprilTagDetector(camera.camera_id,camera_params=camera.parameters) for camera in cameras]


    # Create a streaming task for each camera
    streaming_tasks = [StreamingTask(f"Camera {camera.camera_id} Streaming Task") for camera in cameras]
    # Start the streaming tasks
    for task in streaming_tasks:
        task.start()

    prev_time = time.time()
    running = True
    while running:
        current_time = time.time()

        tables.setDouble("sensors3140/timestamp", current_time)


        dt = current_time - prev_time
        sleep_time = 0.033 - dt
        if sleep_time > 0:
            time.sleep(sleep_time)
        prev_time = current_time

        for camera,at_detector, stream in zip(cameras,at_detectors, streaming_tasks):
            camera: sensors3140.Camera
            at_detector: AprilTagDetector

            frame_data = camera.get_frame()
            frame_data: sensors3140.camera.camera.FrameData
            frame = frame_data.frame
            frame_id = frame_data.frame_id
            prev_time = frame_data.timestamp
            #print(f"Frame ID: {frame_id}, Size: {frame.shape}, Time: {prev_time}")
            tables.setDouble(f"sensors3140/camera{camera.camera_id}/frame_id", frame_id)
            tables.setDouble(f"sensors3140/camera{camera.camera_id}/timestamp", prev_time)
            #tables.setDouble(f"sensors3140/camera{camera.camera_id}/fps", camera.fps)
            tables.setDouble(f"sensors3140/camera{camera.camera_id}/width", camera.width)
            tables.setDouble(f"sensors3140/camera{camera.camera_id}/height", camera.height)

            detections = at_detector(frame_data)
            #print(f"Camera {camera.camera_id} detected {len(detections)} tags")
            #for detection in detections:
            #    print(f"    Id: {detection['id']} Distance: {detection['distance']:.2f}m, Bearing: {detection['bearing']:.2f}°, Azimuth: {detection['azimuth']:.2f}°")
                #print(f"    Pose: {detection['pose']}")
            #    print(f"    Pose Decomposed: {detection['pose_decomposed']}")

            stream.add_input(frame_data)


            # Process the frame here
            if args.display:
                # Display apriltag boxes
                frame = display_apriltag_boxes(frame, detections)

                frame = display_apriltag_pose(frame, detections)

                cv2.imshow(f"Camera {camera.camera_id}", frame)
            
                # Handle quit condition
                key = cv2.waitKey(1)

                if key == ord('q'):
                    running = False

                
        if args.map:
            # Create the map display if it doesn't exist
            if map_display is None:
                map_display = map.LiveMapDisplay("2025-reefscape")
                map_display.load()
                map_display.set_robot_size(0.74, 0.74)

            # update the detected tags
            detected_tags = []
            for detector in at_detectors:
                detected_tags += detector.get_detected_tags()

            map_display.set_detected_tags(detected_tags)

            map_display: map.LiveMapDisplay

            map_display.display()



if __name__ == "__main__":
    main()