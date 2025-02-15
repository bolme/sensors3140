import sensors3140 as sensors3140
import os
import json
import argparse
import cv2
from sensors3140.apriltag.detector import AprilTagDetector
from sensors3140.camera.streaming_task import StreamingTask

import sensors3140.tables.network_tables as nt

import sensors3140.apriltag.maps.map as map

import time

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Sensors3140")

    parser.add_argument("--config", type=str, default="config.json", help="Path to the configuration file")
    parser.add_argument("--display" , action="store_true", help="Display the camera feed", default=False)
    parser.add_argument("--map", action="store_true", help="Display the map", default=False)
    return parser.parse_args()



def main():

    tables = nt.NetworkTablesManager("127.0.0.1")

    # wait for network tables to connect
    for _ in range(100): 
        if tables.is_connected():
            break
        time.sleep(0.1)

    if not tables.is_connected():
        print("WARNING: Failed to connect to NetworkTables")
    else:
        print("Connected to NetworkTables")


    map_display = None

    # Parse command line arguments
    args = parse_args()

    cameras = []

    # Scan the configuration directory for camera configurations
    files = os.listdir(sensors3140.sensors3140_directory)
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
            print(f"Camera {camera.camera_id} detected {len(detections)} tags")
            for detection in detections:
                print(f"    Id: {detection['id']} Distance: {detection['distance']:.2f}m, Bearing: {detection['bearing']:.2f}°, Azimuth: {detection['azimuth']:.2f}°")
                #print(f"    Pose: {detection['pose']}")
                print(f"    Pose Decomposed: {detection['pose_decomposed']}")

            stream.add_input(frame_data)


            # Process the frame here
            if args.display:
                # Display apriltag detections
                for detection in detections:
                    # Draw a bounding box around the tag
                    cv2.polylines(frame, [detection['corners'].astype(int)], True, (0, 255, 0), 2)
                    # Draw the tag ID
                    center_x = int(detection['corners'][:, 0].mean())
                    center_y = int(detection['corners'][:, 1].mean())
                    cv2.putText(frame, str(detection['id']), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
                    cv2.putText(frame, f"{detection['distance']:.2f}m", (center_x, center_y+40), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)

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