import sensors3140 as sensors3140
import os
import json

import time

def main():
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

    time.sleep(3.0)

    while True:
        for camera in cameras:
            frame, frame_id, prev_time = camera.get_frame()
            print(f"Frame ID: {frame_id}, Size: {frame.shape}, Time: {prev_time}")

            # Process the frame here
            time.sleep(1)



            

if __name__ == "__main__":
    main()