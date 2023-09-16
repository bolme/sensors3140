from rplidar import RPLidar
import time
import threading
import sys
from networktables import NetworkTables
import socket
import time
import traceback
import numpy as np


DEFAULT_ROI_LOWER = -10.0
DEFAULT_ROI_UPPER = 10.0

sensorShutdownTimeout = False

lidar = RPLidar('/dev/ttyUSB0')
stop_flag = False

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

if sensorShutdownTimeout == True:
    timer = threading.Timer(3.0, close_lidar)
    timer.start()

hostname=socket.gethostname()
ipaddr=socket.gethostbyname(hostname)

host_ip = socket.gethostbyname("10.31.40.2")

print("Connecting to","10.31.40.2",'  ip:',host_ip)

NetworkTables.initialize(server=host_ip)

at = NetworkTables.getTable("rplidar")
sensor_id = "rplidar"
port = "/dev/ttyUSB0"

print("Port: ", port)

sensor_table = at.getSubTable(sensor_id)
sensor_table.putString('sensor_name',hostname)
sensor_table.putString('sensor_ip',ipaddr)
sensor_table.putString('status',"starting")

#rpl = None
rpl=lidar
try:
    # Connect to the sensor and attemp a reset
#    rpl = RPLidar(port)
#    time.sleep(0.2)
#    rpl.stop()
#    time.sleep(0.2)
#    rpl.reset()
#    rpl.clean_input()

    # Get the basic device info
    print("Attempting to get sensor health.")
    health = rpl.get_health()
    print("Health Recieved: ", health)
    print("Attempting to get sensor info.")
    device_info = rpl.get_info()
    print("Info Recieved!: Connected.")
    sensor_table.putString('status',"connected")

    # Add the device info to the network table
    sensor_table.putString('model',device_info['model'])
    sensor_table.putString('firmware',device_info['firmware'])
    sensor_table.putString('hardware',device_info['hardware'])
    sensor_table.putString('serial_number',device_info['serialnumber'])
    sensor_table.putString('health',health[0])
    sensor_table.putNumber('roi_angle_lower',-15)
    sensor_table.putNumber('roi_angle_upper',15)

    # Run a loop and get all the scans
    print("Sensor running...")
    prev_time = time.time()
    for each in rpl.iter_scans():
        current_time = time.time()
        data = np.array(each)

        # Convert angles to -180 to 180
        select = data[:,1] > 180
        data[select,1] -= 360.0
        
        # Convert from mm to meters
        data[:,2] = data[:,2] / 1000.0

        # Sort the data by angle
        data = list(data)
        data.sort(key=lambda x: x[1])
        data = np.array(data,dtype=np.float32)
        
        # Print some of the values
        #if options.verbose:
        #    for row in data[::10]:
        #        print('A: %0.1f  D: %0.1f'%(row[1],row[2]))
        
        for row in data[::10]:
                print('A: %0.1f  D: %0.1f'%(row[1],row[2]))
                
        # Find the global min, max, and direction to the nearest object
        select = (data[:,2] != 0)
        min_distance = 0.0
        max_distance = 0.0
        min_distance_angle = 0.0

        if select.sum() > 0:
            tmp = data[select]
            min_distance = tmp[:,2].min()    
            max_distance = tmp[:,2].max()  
            min_distance_angle = tmp[tmp[:,2]==min_distance,1].min()
        
        # Find the min, max and direction for the region of interest
        roi_lower = sensor_table.getNumber('roi_angle_lower',DEFAULT_ROI_LOWER)
        roi_upper = sensor_table.getNumber('roi_angle_upper',DEFAULT_ROI_UPPER)
        select = (data[:,2] != 0) & \
                 (data[:,1] > roi_lower) & \
                 (data[:,1] < roi_upper)
        roi_min_distance = 0.0
        roi_max_distance = 0.0
        roi_min_distance_angle = 0.0

        if select.sum() > 0:
            tmp = data[select]
            roi_min_distance = tmp[:,2].min()    
            roi_max_distance = tmp[:,2].max()  
            roi_min_distance_angle = tmp[tmp[:,2]==roi_min_distance,1].min()

        # Update the network tables
        flat = data.flatten()
        sensor_table.putNumber('point_count',flat.shape[0]/3)
        sensor_table.putNumberArray('data_array',flat)
        scan_time = current_time - prev_time
        frame_rate = 1.0/scan_time
        sensor_table.putNumber('scan_time',scan_time)
        sensor_table.putNumber('frame_rate',frame_rate)
        sensor_table.putNumber('min_distance',min_distance)
        sensor_table.putNumber('max_distance',max_distance)
        sensor_table.putNumber('min_distance_angle',min_distance_angle)
        sensor_table.putNumber('roi_min_distance',roi_min_distance)
        sensor_table.putNumber('roi_max_distance',roi_max_distance)
        sensor_table.putNumber('roi_min_distance_angle',roi_min_distance_angle)
        sensor_table.putString('status',"running")


        #print("Points: %d  Scantime: %.2f"%(len(flat)/3,scan_time))
        #print(data[:10,1])
        
        prev_time = current_time

except KeyboardInterrupt:
    # Ctrl-C used to stop the application
    sensor_table.putString('status',"stopped")

except:
    # Encountered another error.  Try to inform network table 
    # that there was a crash.
    sensor_table.putString('status',"crashed")
    traceback.print_exc()
    print("Exception encountered.  Shutting down.")
    time.sleep(0.5)

if rpl != None:
    print('Disconnecting from rplidar sensor.')
    rpl.stop()
    rpl.disconnect()
    time.sleep(1)

print("Finished.")




#for measurment in enumerate(lidar.iter_measurments()):
#    print("quality: ", measurment[1][1], "angle: ", measurment[1][2], "distance: ", measurment[1][3])
