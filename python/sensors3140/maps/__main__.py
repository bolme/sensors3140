import cv2
import numpy as np
import json
import argparse
import os

def parse_args():
    parser = argparse.ArgumentParser(description='Display image with AprilTags overlayed')
    parser.add_argument('--game_id', type=str, help='Path to image file', default='2025-reefscape')
    return parser.parse_args()

if __name__ == "__main__":

    game_id = parse_args().game_id

    # get the location of this module
    image_path = os.path.join(os.path.dirname(__file__), 'images', f"{game_id}.jpg")
    json_path = os.path.join(os.path.dirname(__file__), 'data', f"{game_id}.json")

    img = cv2.imread(image_path)
    assert img is not None, f"Could not load image {game_id}.jpg"

    with open(json_path) as f:
        data = json.load(f)

    # Get the field dimensions
    field_width = data['field']['width']
    field_length = data['field']['length']
    image_width = img.shape[1]
    image_height = img.shape[0]

    # Plot the april tag locations
    for tag in data['tags']:
        id = tag['ID']
        x = tag['pose']['translation']['x']
        y = tag['pose']['translation']['y']
        z = tag['pose']['translation']['z']

        rotation = tag['pose']['rotation']['quaternion']

        # Scale the x and y values to the image size
        # Flip the y axis
        y = field_width - y
        x = x / field_length * image_width
        y = y / field_width * image_height

        # Draw a circle at the tag location
        cv2.circle(img, (int(x), int(y)), 10, (0, 0, 255), -1)

        # Add a label with the tag ID
        text_size = cv2.getTextSize(str(id), cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
        text_x = int(x - text_size[0] / 2)
        text_y = int(y - text_size[1] / 2)
        if text_y < image_height / 2:
            text_y += text_size[1] + 20
        else:
            text_y -= text_size[1] - 10

        cv2.putText(img, str(id), (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

    print(f"Field dimensions: {field_width} x {field_length}")

    # Display the image in an opencv window
    window_name = 'AprilTag Overlay'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.imshow(window_name, img)
    key = cv2.waitKey(30000)