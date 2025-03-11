import argparse
import json
import os
import re
import time
from typing import List, Dict, Any

import cv2
import numpy as np
import psutil
import logging  # Add logging import
import traceback

import sensors3140
import sensors3140.tables.network_tables as nt
from sensors3140.apriltags.detector import AprilTagDetector
from sensors3140.maps import live_map_display
from sensors3140.camera.streaming_task import StreamingTask

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
_logging = logging.getLogger(__name__)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sensors3140")

    parser.add_argument("--config", type=str, default="config.json", help="Path to the configuration file")
    parser.add_argument("--display" , action="store_true", help="Display the camera feed", default=False)
    parser.add_argument("--map", action="store_true", help="Display the map", default=False)
    return parser.parse_args()

def display_apriltag_boxes(img: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
    for detection in detections:
        # Draw the tag outline
        corners = detection['corners']
        for i in range(len(corners)):
            cv2.line(img, tuple(corners[i-1, :].astype(int)), tuple(corners[i, :].astype(int)), (0, 255, 0), 2)

        # Draw the center
        center = detection['center']
        cv2.circle(img, tuple(center.astype(int)), 5, (0, 255, 0), -1)

        # Draw the ID and distance
        cv2.putText(img, f"ID: {detection['id']}", tuple(center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        meter_to_inch = 39.3701
        cv2.putText(img, f"Dist: {detection['distance']:.2f} m or {meter_to_inch*detection['distance']:.1f} inches", (center[0].astype(int), center[1].astype(int) + 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return img

def display_apriltag_pose(img: np.ndarray, detections: List[Dict[str, Any]]) -> np.ndarray:
    for detection in detections:
        pose = detection['pose']

        tag_translation = pose[0:3, 3]
        tag_rotation = pose[0:3, 0:3]

        camera_params = detection['camera_params']

        # Tag center is the translation in homogenous coordinates
        tag_center = np.array([[tag_translation[0]], [tag_translation[1]], [tag_translation[2]], [1.0]])

        # Compute the 3x4 camera projection matrix
        kx, ky, cx, cy = camera_params

        camera_matrix = np.array([[kx, 0, cx], [0, ky, cy], [0, 0, 1]])
        transform = np.eye(4)
        projection_matrix = np.dot(camera_matrix, transform[:3, :])

        transformed_center = np.dot(projection_matrix, tag_center)

        image_coords = transformed_center[0:2] / transformed_center[2]

        # Draw the tag center as a yellow circle outline
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

def initialize_network_tables() -> nt.NetworkTablesManager:
    network_config_path = os.path.join(sensors3140.sensors3140_directory, "network.json")
    if not os.path.exists(network_config_path):
        _logging.info(f"Network configuration not found. Creating at {network_config_path}")
        data = {
            "network_table_ip": "10.31.40.2",
            "network_table_port": 1735
        }
        with open(network_config_path, "w") as f:
            json.dump(data, f, indent=4)

    with open(network_config_path, "r") as f:
        data = json.load(f)
        _logging.info("Loaded network configuration")

    tables = nt.NetworkTablesManager(data["network_table_ip"])
    for _ in range(50): 
        if tables.is_connected():
            break
        time.sleep(0.1)
    if not tables.is_connected():
        _logging.warning("Failed to connect to NetworkTables")
    else:
        _logging.info("Connected to NetworkTables")
    return tables

def initialize_cameras(tables: nt.NetworkTablesManager) -> List[sensors3140.Camera]:
    files = os.listdir(sensors3140.sensors3140_directory)
    cameras = []
    for file in files:
        match = re.match(r"camera_(\d+)\.json", file)
        if match:
            with open(os.path.join(sensors3140.sensors3140_directory, file), "r") as f:
                data = json.load(f)
            width,height = data["frame_size"]
            camera = sensors3140.Camera(frame_stas=False,width=width,height=height,**data)
            cameras.append(camera)
            camera.initialize_network_tables(tables)
            _logging.info(f"Created camera {camera.camera_id}")
    return cameras

def update_system_metrics(tables: nt.NetworkTablesManager) -> float:
    current_time = time.time()
    tables.setDouble("sensors3140/timestamp", current_time)
    load_average = os.getloadavg()
    tables.setDoubleArray("sensors3140/load_average", load_average)
    memory_usage = psutil.virtual_memory().percent
    tables.setDouble("sensors3140/memory_usage", memory_usage)
    return current_time

def process_camera_frames(cameras: List[sensors3140.Camera], at_detectors: List[AprilTagDetector], streaming_tasks: List[StreamingTask], tables: nt.NetworkTablesManager, args: argparse.Namespace) -> None:
        running = True
        map_display = None
        if args.map:
            map_display = live_map_display.LiveMapDisplay("2025-reefscape")
            map_display.load()
            map_display.set_robot_size(0.74, 0.74)
        
        while running:
            current_time = update_system_metrics(tables)
            detected_tags = []
            
            for camera, at_detector, stream in zip(cameras, at_detectors, streaming_tasks):
                frame_data = camera.get_frame()
                if frame_data is not None and frame_data.frame is not None:
                    try:
                        frame = frame_data.frame
                        camera.update_network_tables(tables)
                        detections = at_detector(frame_data)
                        detected_tags.extend(detections)
                        stream.add_input(frame_data)
                        if args.display:
                            frame = display_apriltag_boxes(frame, detections)
                            frame = display_apriltag_pose(frame, detections)
                            cv2.imshow(f"Camera {camera.camera_id}", frame)
                    except Exception as e:
                        traceback.print_exc()

                if not tables.is_connected():
                    print("Not connected to NetworkTables")
                    _logging.error("Not connected to NetworkTables")

            if args.map and map_display:
                map_display.set_detected_tags(detected_tags)
                map_display.display()
                
            if cv2.waitKey(1) == ord('q'):
                running = False
                
            time.sleep(max(0, 0.033 - (time.time() - current_time)))

def main() -> None:
    try:
        args = parse_args()
        tables = initialize_network_tables()
        cameras = initialize_cameras(tables)
        camera_ids = [camera.camera_id for camera in cameras]
        tables.setDoubleArray("sensors3140/camera_ids", camera_ids)
        time.sleep(3)
        at_detectors = [AprilTagDetector(camera.camera_id, camera_params=camera.camera_params, dist_coeff=camera.dist_coeffs) for camera in cameras]
        streaming_tasks = [StreamingTask(camera.camera_id) for camera in cameras]
        for task in streaming_tasks:
            task.start()
        process_camera_frames(cameras, at_detectors, streaming_tasks, tables, args)
    except KeyboardInterrupt:
        _logging.info("Program interrupted by user")
    finally:
        # Clean up resources
        _logging.info("Shutting down...")
        for task in streaming_tasks:
            task.stop()
        for camera in cameras:
            camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()