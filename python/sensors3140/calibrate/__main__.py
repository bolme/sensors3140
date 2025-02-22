import cv2
import apriltag
import numpy as np
import sensors3140
import json
import os
import optparse
import time

def parseOptions():
    useage = "python3 -m sensor3140.calibrate.calibrate_camera [options]"
    parser = optparse.OptionParser(useage,version=sensors3140.__version__)

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
    

    options,args = parser.parse_args()

    try:
        options.camera_id = int(options.camera_id)
    except:
        pass

    # make sure rotate is an acceptable value
    assert options.rotate in [0,90,180,270]

    return options, args




options,args = parseOptions()

is_file = False
if os.path.isfile(options.camera_id):
    print(f"Loading video from the file: {options.camera_id}")
    is_file = True

print(dir(options))

# Generate a mapping from point codes to a target location

obj_p_map = {}

idx = 0
for j in range(5):
    for i in range(6):
        x = 0.04*i
        y = 0.04*j
        z = 0.0
        obj_p_map[idx] = (x,y,z)
        idx += 1


opts = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(opts)


cap = cv2.VideoCapture(options.camera_id)
if not is_file:
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,options.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,options.height)
    cap.set(cv2.CAP_PROP_FPS,options.fps)


i = 0

img_points = []
obj_points = []


size = (0,0)
try:
    prev_mtx = None
    error_vals = []
    while cap.grab():
    
        i += 1
        flag, image = cap.retrieve()

        if options.rotate != 0:
            if options.rotate == 90:
                image=cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            if options.rotate == 180:
                image=cv2.rotate(image, cv2.ROTATE_180)
            if options.rotate == 270:
                image=cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)

        print(f"Detected {len(results)} tags.")

        if options.display:
            for each in results:
                cX,cY = int(each.center[0]), int(each.center[1])
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        size = gray.shape[::-1]

        img = []
        obj = []
        if i % 15 == 0 and len(results) > 8:
            for each in results:
                img.append(each.center)
                obj.append(obj_p_map[each.tag_id])

            img_points.append(np.array(img,dtype=np.float32))
            obj_points.append(np.array(obj,dtype=np.float32))
            
            if len(img_points) > 3:
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

                if prev_mtx is None:
                    prev_mtx = mtx
                else:
                    diff_mtx = prev_mtx - mtx
                    prev_mtx = mtx
                    err = np.linalg.norm(diff_mtx,ord=2)
                    error_vals.append(err)
                    error_vals = error_vals[-10:]
                    if len(error_vals) > 8:
                       # Count the total number of points in img_points
                       total_points = 0
                       for points in img_points:
                           total_points += len(points)
                           

                       print(f"Average Error: {np.mean(error_vals)}   Total Points: {len(img_points)} {total_points}")
                       if np.mean(error_vals) < 1.0:
                           break

        if options.display:
            cv2.imshow('img',image)
            key = cv2.waitKey(3)
            if key != -1:
                print('key:',key)
                break
except KeyboardInterrupt:
    print("Finishing up.")
        

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)


result = {
    'camera_id':options.camera_id,
    'frame_size':[options.width,options.height],
    'fps':options.fps,
    'exposure':options.exposure,
    'gain':options.gain,
    'rotate':options.rotate,
    'matrix':list(mtx.flatten()),
    'parameters':[mtx[0,0],mtx[1,1],mtx[0,2],mtx[1,2]],
    'distortion':list(dist.flatten()),
    }


print(json.dumps(result,indent=4))
if options.outfile:
    json.dump(result,open(options.outfile,'w'),indent=4)
else:
    config_directory = sensors3140.sensors3140_directory
    # Create the directory if it doesn't exist
    if not os.path.exists(config_directory):
        os.makedirs(config_directory)
    
    if is_file:
        # get the filename with the extension removed
        camera_name = os.path.splitext(os.path.basename(options.camera_id))[0]
        camera_save_path = os.path.join(config_directory,f"{camera_name}.json") 
    else:
        camera_save_path = os.path.join(config_directory,f"camera_{options.camera_id}.json")

    # If the file exists then back it up

    if os.path.exists(camera_save_path):
        # Get the file modification datetime
        timestamp = time.strftime('%Y-%m-%d_%H-%M-%S',time.localtime(os.path.getmtime(camera_save_path)))
        backup_path = camera_save_path.replace('.json',f'_{timestamp}.json')
        os.rename(camera_save_path,backup_path)

    json.dump(result,open(camera_save_path,'w'),indent=4)




    # "parameters": [
    #     513.5692261981907,
    #     511.9628279267013,
    #     324.45659369977324,
    #     242.15586616864744
    # ],
    # "distortion": [
    #     0.007654730212483044,
    #     0.14223667786170405,
    #     0.0038073770122510294,
    #     0.0012131524239662,
    #     -0.37253895909847246
    # ]

