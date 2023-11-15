# Fixed Issues: 

* Issues:
  * Lidar performed two scans before crashing - This was caused becaues the Raspberry Pi would boot faster than the RoboRio and start sending data, eventually overflowing it's buffer and crashing the program.
* Fixes:
  * Made a while loop that pings the IP address given in the rplidar.json file for the RoboRio until it gets a 0% packet loss response with 1 packet. Once recieved it will perform as before. 

<hr>

# RPLidar Code

This script is a stand-alone version of the RPLidar Code.

# Setting up Script:

 1. Make sure everything is up to date on the Pi.
```
sudo apt-get update
sudo apt update
sudo apt full-upgrade
``` 
 2. Clone the StandAloneRPLidarCode branch into /home/dev3140 on Raspberry Pi.
```
git clone -b StandAloneRPLidarCode --single-branch https://github.com/FRC-Team-3140/sensors3140 RPLidar
```
 3. Clone the sensors3140 main branch into /home/dev3140 on Raspberry Pi and configure.
```
git clone https://github.com/FRC-Team-3140/sensors3140
sudo apt install gcc cmake libopencv-dev python3-dev python3-opencv python3-numpy python3-scipy python3-sklearn python3-skimage python3-pandas python3-pip
cd sensors3140
python -m pip install -e .
```    
 4. Install required dependencies.
```
pip install pynetworktables rplidar-roboticia
```
 5. Ensure that the script executes properly by manually executing it with the terminal.

__Note:__ Some files require direct file paths, especially the lines for setting up the service and crontab. Make sure these are correct when you set up a new pi and user. 

# Running Program on Start:

There are multiple ways to do this. The two easiest ways are: 
 * Setting up a service
 * Utilizing the Raspberry Pi's Crontab

## Setting up script with Service:

To set up a service to run on Raspberry Pi boot, run the following lines below: 
```
sudo cp config/rplidar.service /etc/systemd/system/rplidar.service
```
```
sudo chmod 664 /etc/systemd/system/rplidar.service
```

## Setting up the script with Chrontab: 

Use the command below to edit the crontab.
```
crontab -e
```
Add the line to the end of the Crontab.
```
@reboot python /home/dev3140/RPLidar/__main__.py --c=/home/dev3140/RPLidar/rplidar.json
```
- If you want the script to output all of the sensor readings you can use the following line.
```
@reboot python /home/dev3140/RPLidar/__main__.py --c=/home/dev3140/RPLidar/rplidar.json -v
```
- To ensure that the script is working correctly, you can have the crontab put the output of the script into a log file. Like this:
```
@reboot python /home/dev3140/RPLidar/__main__.py --c=/home/dev3140/RPLidar/rplidar.json >> /home/dev3140/RPLidar/RPLidarScript.log 2>&1
```
__Note:__ This log file will continue to grow in size the longer you run the lidar with the -v tag, so it's recommended to not use the -v tag for more than testing if you have the crontab configured to output the script output into a log file.
