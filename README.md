# RPLidar Code

This script is a stand-alone version of the RPLidar Code.

# Get Script:

 1. Clone repository into root on Raspberry pi.
 2. Ensure that script executes properly by manually executing it with the terminal.
 __Note:__ Some files require direct file paths, especially the lines for setting up the service and crontab. Make sure these are correct when you set up a new pi. 

# Running Program on Start:

There are multiple ways to do this. The two easiest ways are: 
 * Setting up a service
 * Utilizing the Raspberry Pi's Crontab

## Setting up script with Service:

To set up a service to run on Raspberry Pi start, run the following lines below: 
```
sudo cp config/rplidar.service /etc/systemd/system/rplidar.service
```
```
sudo chmod 664 /etc/systemd/system/rplidar.service
```

## Setting up script with Chrontab: 

Use the command below to edit the crontab.
```
crontab -e
```
Add the line to the end of the Crontab.
```
@reboot python3 /home/dev3140/RPLidar/__main__.py --c=/home/dev3140/RPLidar/rplidar.json
```
- If you want the script to output all of the sensor readings you can use the following line.
```
@reboot python3 /home/dev3140/RPLidar/__main__.py --c=/home/dev3140/RPLidar/rplidar.json -v
```
- To ensure that the script is working correctly, you can have the crontab put the output of the script into a log file. Like this:
```
@reboot python3 /home/dev3140/RPLidar/__main__.py --c=/home/dev3140/RPLidar/rplidar.json >> /home/dev3140/RPLidar/RPLidarScript.log 2>&1
```
__Note:__ This log file will continue to grow in size the longer you run the lidar with the -v tag, so it's recommended to not use the -v tag for more than testing.
