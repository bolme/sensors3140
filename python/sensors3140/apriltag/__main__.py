import cv2
import apriltag
import numpy as np
#import os
#import skimage.io
from networktables import NetworkTables
import json
import time
import socket
import optparse
import sensors3140


def parseOptions():
    useage = "python3 -m sensor3140.calibrate.calibrate_camera [options]"
    parser = optparse.OptionParser(useage,version=sensors3140.__version__)

    parser.add_option("-c", "--config", dest="config", default='apriltag.json',
                  help="write report to this json FILE", metavar="FILE")

    parser.add_option("-d", "--display", dest="display", default=False, action='store_true',
                  help="display a window with live video")

    parser.add_option("-s", "--sample", dest="sample", default=False, action='store_true',
                  help="Save a sample image every few seconds to 'sample.png'.  This image can help with debuging and image quality issues.")

    options,args = parser.parse_args()

    return options, args


options, args = parseOptions()

hostname=socket.gethostname()
ipaddr=socket.gethostbyname(hostname)

# Configure network tables
config = json.load(open(options.config,'r'))
calib = json.load(open(config['calibration'],'r'))

host_ip = socket.gethostbyname(config['networktables_host'])

print("Connecting to",config['networktables_host'],'  ip:',host_ip)

NetworkTables.initialize(server=host_ip)

at = NetworkTables.getTable("apriltags")
sensor_id = config['sensor_id']
sensor_table = at.getSubTable(sensor_id)
sensor_table.putString('sensor_name',hostname)
sensor_table.putString('sensor_ip',ipaddr)
sensor_table.putNumberArray('frame_size',calib['frame_size'])
sensor_table.putNumber('tag_size',config['tag_size'])
sensor_table.putString('tag_family',config['tag_family'])

tags_table = sensor_table.getSubTable('tags')

camera_param = calib['parameters']

det_options = apriltag.DetectorOptions(families="tag36h11")
detector = apriltag.Detector(det_options)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,calib['frame_size'][0])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,calib['frame_size'][1])
cap.set(cv2.CAP_PROP_FPS,calib['fps'])
cap.set(cv2.CAP_PROP_EXPOSURE,calib['exposure'])
cap.set(cv2.CAP_PROP_GAIN,calib['gain'])

current_time = time.time()
last_update = time.time()
while cap.grab():
    flag, image = cap.retrieve()
    prev_time = current_time
    current_time = time.time()

    # Display information on camera speed.
    update = False
    if current_time - last_update >= 2.0:
        update = True
        last_update = current_time
        print('***** est_fps:',1.0/(current_time-prev_time),'  target_fps:',cap.get(cv2.CAP_PROP_FPS),"  exposure:",cap.get(cv2.CAP_PROP_EXPOSURE),"  gain:",cap.get(cv2.CAP_PROP_GAIN), "*****")

    if not flag:
        print('error')
        continue
    else:
        #print(frame.shape)
        start = time.time()
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)
        detect_time = time.time() - start

        h,w = gray.shape
        sensor_table.putNumberArray('frame_size',[w,h])
        for r in results:
            #print(r.center)
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # draw the bounding box of the AprilTag detection
            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            # draw the tag family on the mage

            pose = detector.detection_pose(r,camera_param,0.16)

            trans = pose[0][0:3,3]

            tagFamily = r.tag_family.decode("utf-8")
            dist = np.sqrt((trans**2).sum())
            bearing = np.arctan2(trans[0],trans[2]) * 180.0 / np.pi
            azimuth = np.arctan2(-trans[1],trans[2]) * 180.0 / np.pi

            if options.display or (update and options.sample):
                cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
                cv2.putText(image,"ID: %s"%r.tag_id, (cX, cY + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image,"DIST: %0.3fm"%dist, (cX, cY + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image,"BEARING: %0.1fdeg"%bearing, (cX, cY + 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(image,"AZIMUTH: %0.1fdeg"%azimuth, (cX, cY + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if options.sample:
                    cv2.imwrite('apriltag_sample.png',image)

            #print(pose)

            # Post the latest tag information to the network tables    
            tag = tags_table.getSubTable('tag_%03d'%r.tag_id)
            tag.putNumberArray('center',r.center)
            tag.putNumberArray('corners',r.corners.flatten())
            tag.putNumber('last_update',current_time)
            tag.putNumberArray('location',pose[0][0:3,3])
            tag.putNumberArray('pose',pose[0].flatten())

            tag.putNumber('distance',dist)
            tag.putNumber('bearing',bearing)
            tag.putNumber('azimuth',azimuth)


            print("TID: %3d  DIST: %6.3f  BRG: %4.1f  AZ: %4.1f"%(r.tag_id,dist,bearing,azimuth))


        sensor_table.putNumber('last_update',current_time)
        sensor_table.putNumber('detect_time',detect_time)
        sensor_table.putNumber('update_hz',1.0/(current_time-prev_time))

        if options.display:
            cv2.putText(image,"Mode: %d X %d @ %d fps"%(calib['frame_size'][0],calib['frame_size'][1],calib['fps'],), (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(image,"AT Speed: TIME: %0.1f ms  FPS: %0.2f"%(1000*detect_time,1.0/(current_time-prev_time)), (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('img',image)
            key = cv2.waitKey(3)
            if key != -1:
                break

