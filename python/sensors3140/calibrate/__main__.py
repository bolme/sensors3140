import cv2
import apriltag
import numpy as np
import sensors3140
import json
import os
import optparse
import time
from datetime import datetime


def parseOptions():
    useage = "python3 -m sensor3140.calibrate.calibrate_camera [options]"
    parser = optparse.OptionParser(useage, version=sensors3140.__version__)

    parser.add_option("-o", "--outfile", dest="outfile", default=None,
                  help="Override the default outfile location.", metavar="FILE")

    parser.add_option("-d", "--display", dest="display", default=False, action='store_true',
                  help="display a window with live video")
    
    parser.add_option("--camera-id", dest="camera_id", default='0', type="str",
                  help="ID of the camera to use.")

    parser.add_option("--width", dest="width", default=640, type="int",
                  help="Width of the frame.")

    parser.add_option("--height", dest="height", default=480, type="int",
                  help="height of the frame.")

    parser.add_option("--fps", dest="fps", default=15, type="int",
                  help="frames per second.")

    parser.add_option("--exposure", dest="exposure", default=1.0, type="float",
                  help="Exposure modifier.")

    parser.add_option("--gain", dest="gain", default=-1.0, type="float",
                  help="Gain modifier.")

    parser.add_option("--rotate", dest="rotate", default=0, type="int",
                  help="Rotate the camera frame 0, 90, 180, 270.")
    
    options, args = parser.parse_args()

    try:
        options.camera_id = int(options.camera_id)
    except:
        pass

    # make sure rotate is an acceptable value
    assert options.rotate in [0, 90, 180, 270]

    return options, args


def create_object_points_map():
    """Generate a mapping from point codes to a target location"""
    obj_p_map = {}
    idx = 0
    for j in range(5):
        for i in range(6):
            x = 0.04 * i
            y = 0.04 * j
            z = 0.0
            obj_p_map[idx] = (x, y, z)
            idx += 1
    return obj_p_map


def setup_camera(options, is_file):
    """Setup the camera with the specified options"""
    cap = cv2.VideoCapture(options.camera_id)
    if not is_file:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, options.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, options.height)
        cap.set(cv2.CAP_PROP_FPS, options.fps)
    return cap


def process_frame(image, options, detector, obj_p_map):
    """Process a single frame to detect tags"""
    if options.rotate != 0:
        if options.rotate == 90:
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        if options.rotate == 180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        if options.rotate == 270:
            image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)

    if options.display:
        for each in results:
            cX, cY = int(each.center[0]), int(each.center[1])
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

    size = gray.shape[::-1]
    return image, gray, results, size


def save_calibration_result(result, options, is_file):
    """Save the calibration result to a file"""
    if options.outfile:
        json.dump(result, open(options.outfile, 'w'), indent=4)
        print(f"Calibration saved to: {options.outfile}")
    else:
        config_directory = sensors3140.sensors3140_directory
        # Create the directory if it doesn't exist
        if not os.path.exists(config_directory):
            os.makedirs(config_directory)
        
        if is_file:
            # get the filename with the extension removed
            camera_name = os.path.splitext(os.path.basename(options.camera_id))[0]
            camera_save_path = os.path.join(config_directory, f"{camera_name}.json") 
        else:
            camera_save_path = os.path.join(config_directory, f"camera_{options.camera_id}.json")

        # If the file exists then back it up
        if os.path.exists(camera_save_path):
            # Get the file modification datetime
            timestamp = time.strftime('%Y-%m-%d_%H-%M-%S', time.localtime(os.path.getmtime(camera_save_path)))
            backup_path = camera_save_path.replace('.json', f'_{timestamp}.bak.json')
            os.rename(camera_save_path, backup_path)
            print(f"Backed up existing calibration to: {backup_path}")

        json.dump(result, open(camera_save_path, 'w'), indent=4)
        print(f"Calibration saved to: {camera_save_path}")


def collect_calibration_points(results, obj_p_map, img_points, obj_points):
    """Collect calibration points from detected tags"""
    img = []
    obj = []
    for each in results:
        if each.tag_id in obj_p_map:
            img.append(each.center)
            obj.append(obj_p_map[each.tag_id])

    if len(img) > 8:
        img_points.append(np.array(img, dtype=np.float32))
        obj_points.append(np.array(obj, dtype=np.float32))
    return img_points, obj_points

def calibrate_camera(obj_points, img_points, size):
    """Calibrate the camera using collected points"""
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
    return mtx, dist

def display_frame(image, results):
    """Display the frame with detected tags"""
    for each in results:
        cX, cY = int(each.center[0]), int(each.center[1])
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    cv2.imshow('Camera Calibration', image)

class CameraCapture:
    def __init__(self, options, is_file):
        self.cap = cv2.VideoCapture(options.camera_id)
        if not is_file:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, options.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, options.height)
            self.cap.set(cv2.CAP_PROP_FPS, options.fps)

    def __enter__(self):
        return self.cap

    def __exit__(self, exc_type, exc_value, traceback):
        self.cap.release()
        if exc_type is not None:
            print(f"Exception: {exc_value}")

def save_images(saved_images, mtx, dist, camera_id):
    """Save original and undistorted images to a directory"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.expanduser(f"~/sensors3140/camera_{camera_id}_images_{timestamp}")
    if not os.path.exists(directory):
        os.makedirs(directory)

    for idx, image in enumerate(saved_images):
        original_path = os.path.join(directory, f"frame_{idx}_original.png")
        undistorted_path = os.path.join(directory, f"frame_{idx}_undistorted.png")

        cv2.imwrite(original_path, image)
        undistorted_image = cv2.undistort(image, mtx, dist)
        cv2.imwrite(undistorted_path, undistorted_image)

    print(f"Saved {len(saved_images)} original and undistorted images to {directory}")

def main():
    options, args = parseOptions()

    is_file = os.path.isfile(options.camera_id)
    print(f"Loading video from the file: {options.camera_id}" if is_file else f"Using camera with ID: {options.camera_id}")

    obj_p_map = create_object_points_map()
    opts = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(opts)

    img_points, obj_points, size = [], [], (0, 0)
    frame_count, last_print_time, error_vals, prev_mtx = 0, time.time(), [], None
    saved_images = []
    last_save_time = time.time()

    print("Starting calibration. Press Ctrl+C to finish early or any key in display window to stop.")
    try:
        with CameraCapture(options, is_file) as cap:
            while cap.grab():
                frame_count += 1
                flag, image = cap.retrieve()

                image, gray, results, size = process_frame(image, options, detector, obj_p_map)

                current_time = time.time()
                if current_time - last_print_time >= 0.5:
                    print(f"Frame {frame_count}: Detected {len(results)} tags, Collected {len(img_points)} calibration samples")
                    last_print_time = current_time

                if current_time - last_save_time >= 3:
                    saved_images.append(image.copy())
                    print(f"Saved image at frame {frame_count}. Total saved images: {len(saved_images)}")
                    last_save_time = current_time

                if frame_count % 15 == 0 and len(results) > 8:
                    img_points, obj_points = collect_calibration_points(results, obj_p_map, img_points, obj_points)
                    if len(img_points) > 3:
                        mtx, dist = calibrate_camera(obj_points, img_points, size)
                        if prev_mtx is None:
                            prev_mtx = mtx
                        else:
                            diff_mtx = prev_mtx - mtx
                            prev_mtx = mtx
                            err = np.linalg.norm(diff_mtx, ord=2)
                            error_vals.append(err)
                            error_vals = error_vals[-10:]
                            if len(error_vals) > 8:
                                total_points = sum(len(points) for points in img_points)
                                if current_time - last_print_time >= 0.5:
                                    print(f"Average Error: {np.mean(error_vals):.4f}   Total Frames: {len(img_points)}   Total Points: {total_points}")
                                    last_print_time = current_time
                                if np.mean(error_vals) < 1.0:
                                    print("Calibration has converged! Finishing up.")
                                    break

                if options.display:
                    display_frame(image, results)
                    if cv2.waitKey(3) != -1:
                        print('Key pressed, stopping calibration.')
                        break

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Finishing calibration.")
    finally:
        if len(img_points) > 3:
            print(f"Finalizing calibration with {len(img_points)} images containing {sum(len(p) for p in img_points)} points")
            mtx, dist = calibrate_camera(obj_points, img_points, size)
            result = {
                'camera_id': options.camera_id,
                'frame_size': [options.width, options.height],
                'fps': options.fps,
                'exposure': options.exposure,
                'gain': options.gain,
                'rotate': options.rotate,
                'matrix': list(mtx.flatten()),
                'parameters': [mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]],
                'distortion': list(dist.flatten()),
                'calibration_date': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            }
            print("\nCalibration Results:")
            print(json.dumps(result, indent=4))
            save_calibration_result(result, options, is_file)
            save_images(saved_images, mtx, dist, options.camera_id)
        else:
            print("Not enough calibration data collected. Try again with more frames.")
        if options.display:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

