from rplidar import RPLidar
import time
import threading
import sys

# :::::::::::::::::::::::::::::::: WARNING ::::::::::::::::::::::::::::::::
# When the lidar shuts down, either with the testTimeout or with Keyboard Interrupt, the script might throw errors.

lidar = RPLidar('/dev/ttyUSB0')
stop_flag = False

testTimeout = 3.0
# Chage to True if you want the sensor to shutdown after a duration.
timeout = False

def close_lidar(): 
    stop_flag = True
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    sys.exit(0)
    
info = lidar.get_info()
print(info)

health = lidar.get_health()
print(health)

time.sleep(2)

if timeout == True:
    timer = threading.Timer(testTimeout, close_lidar)
    timer.start()

for measurment in enumerate(lidar.iter_measurments()):
    print("quality: ", measurment[1][1], "angle: ", measurment[1][2], "distance: ", measurment[1][3])
