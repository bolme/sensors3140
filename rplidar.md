# RPLidar

This simple python program is intended to be run on a coprocessor such as a 
Raspberry Pi 3/4 or NVIDIA Jetson Nano to run an rplidar sensor.  The sensor
will use laser pulses to measure distances in a 360 degree circle around the 
sensor.

The rplidar tool can be run from the command line and the `-h` option will 
display additional configuration options:

From the home directory you will need to install the configuration:
```
cd ~pi
```

```
cp sensors3140/config/rplidar.json ./
```

```
python3 -m sensors3140.rplidar
```

Measurements are communicated through a NetworkTables interface in the 
table `rplidar`.

There is a `rplidar.json` file that configures the tool.  This should be 
updated with the proper ip for the network and the camera device.


## Running as a service: RPLidar

In many cases you will want to boot your sensor and have the rplidar 
process run automatically.  There are many ways to accomplish this.  Here 
we install it as a unit in systemctl.  See these 
[examples](https://www.shellhacks.com/systemd-service-file-example/) 
for more a detailed tutorial or to customize the process.  Here we provide 
a tutorial and files needed for a typical raspberry pi coprocessor 
configuration.

### Configuration Files

 * rplibar.json - This controls basic parameters for the rplidar 
   process.  It is important to verify the following:
    * networktables_host - should be set to the ip of your RoboRio. This is 
      the primary method for communicating detections and your controller 
      applications should read detection data from this table.  (For Romi 
      this should be set to the ip of the development laptop.)
    * sensor_id - controls the name of the subtable name in network tables.  
      By changing this it is possible to use multiple cameras and rplidar 
      coprocessors on the same robot.

To install a basic configuration you can copy the json files to the following 
location.  This assumes the use of the preexisting raspberry pi v2.1 camera 
calibration file.  However, for most accurate results it is recommended that 
you generate the `camera.json` file yourself using the camera calibration tool.

```
cp config/rplidar.json /home/pi/rplidar.json
```

Files to start the lidar on boot can be installed like this:
```
sudo cp config/rplidar.service /etc/systemd/system/rplidar.service
sudo chmod 664 /etc/systemd/system/rplidar.service
```

### Editing the unitfile
The unit file sets the paths and configuration to start up the rplidar 
coprocessor.  It is a good idea to check the username, paths, etc to make 
sure the configuration is correct.

```
sudo nano /etc/systemd/system/rplidar.service
```

### Testing the service
To load the changes into systemctl the daemon needs to be reloaded with 
the following command.

```
sudo systemctl daemon-reload
```

The system can be tested with the following commands:
```
sudo systemctl start rplidar
sudo systemctl stop rplidar
```

### Enable at boot
In order to start the service with the device boots run this command.
```
sudo systemctl enable rplidar
```

### Checking the logs
If there are issues you should be able to check the log with:
```
journalctl -u rplidar
```
