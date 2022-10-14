import cv2
import apriltag
import numpy as np
import sensors3140
import json

import optparse

def parseOptions():
    useage = "python3 -m sensor3140.calibrate.calibrate_camera [options]"
    parser = optparse.OptionParser(useage,version=sensors3140.__version__)

    parser.add_option("-o", "--outfile", dest="outfile", default=None,
                  help="write report to this json FILE", metavar="FILE")

    parser.add_option("-d", "--display", dest="display", default=False, action='store_true',
                  help="display a window with live video")

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

    # make sure rotate is an acceptable value
    assert options.rotate in [0,90,180,270]

    return options, args




options,args = parseOptions()

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


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,options.width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,options.height)
cap.set(cv2.CAP_PROP_FPS,options.fps)


i = 0

img_points = []
obj_points = []


size = (0,0)
try:
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

        if options.display:
            for each in results:
                #print(each.center)
                cX,cY = int(each.center[0]), int(each.center[1])
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

        size = gray.shape[::-1]

        img = []
        obj = []
        if i % 15 == 0 and len(results) > 8:
            for each in results:
                #print(each.tag_id,each.center,obj_p_map[each.tag_id])
                img.append(each.center)
                obj.append(obj_p_map[each.tag_id])

            img_points.append(np.array(img,dtype=np.float32))
            obj_points.append(np.array(obj,dtype=np.float32))

            print("    New Points:",len(img),"Frames: ",len(img_points))

            
            if len(img_points) > 3:
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)
                print('params:',mtx[0,0],mtx[1,1],mtx[0,2],mtx[1,2])

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



