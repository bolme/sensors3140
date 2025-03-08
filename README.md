

Apriltag targets are going to be new for the 2023 First Robotics Competition according to this [announcement](https://www.firstinspires.org/robotics/frc/blog/2022-control-system-reporting-2023-updates-and-beta-testing).  The code provide here is intended to be used as a stand alone [coprocessor](https://docs.wpilib.org/en/stable/docs/software/vision-processing/wpilibpi/using-a-coprocessor-for-vision-processing.html) that may run using a variety of camera modules or webcams on platforms such as Raspberry Pi, Nvidia Jetson, or similar hardware.  Due to supply chain issues these devices may be hard find or too expensive but Libre AML-S905X-CC (Currently Untested) seems to be an easy to obtain and cost effective substitute. 

*First intents to release apriltag detectors that will work with the RoboRio in the near future.  The interface provided by sensors3140 my be updated to improve compatibility for the 2023 season.  See additional info from [chiefdelphi](https://www.chiefdelphi.com/t/photonvision-beta-2023-apriltags/415626).*

*Much of the testing for this library was done on a Romi using a Raspberry Pi camera.  To work with a RoboRio or other camera models some parameters may need to be adjusted.*

# Installation

Installation requires 4 parts.  

Environment will be in : ~/vision

Code will be in: ~/Programming/sensors3140

Configuration files will be in: ~/sensors3140

Start up scripts will be in: systemctl

## Dependencies
A number of dependencies may be required. The following is suggested for Raspberry Pi and similar platforms. 
 * build tools like gcc, cmake
 * Computer vision libraries like libopencv
 * A basic Python 3 environment is recommended with development and some computer vision, imaging, and machine learning tools installed. 

**MacOS Specific Installs**
 * `xcode-select --install`
 * `sudo xcodebuild -license`

On Raspberry Pi and Nvidia Jetsons this software can typically be installed with the apt utility.  Here we suggest getting development libraries which may be required to install some packages from source.  Nvidia Jetson platforms may have specialized installation procedures for libraries like opencv to be accelerated by the the Cuda GPU.

```
sudo apt update
sudo apt install gcc cmake libopencv-dev python3-dev python3-opencv python3-numpy python3-scipy python3-sklearn python3-skimage python3-pandas python3-pip
```


## Install from github
Install the code directly from github.  This should put the code in your python environment but will not allow modification.

***You will still need to install camera.json calibration and apriltag.json files from the github repository.***

```
python3 -m pip install git+https://github.com/FRC-Team-3140/sensors3140
```

## Install from source
Install on a raspberry pi:

```python
python3 -m venv ~/vision
source ~/vision/bin/activate

mkdir Programming
cd Programming
git clone https://github.com/FRC-Team-3140/sensors3140.git
cd sensors3140
python -m pip install -e .
```


This is useful for development.  The pip command install a link to your local copy of the python code which allows the code to be modified and updated easily.

```
git clone https://github.com/FRC-Team-3140/sensors3140.git
cd sensors3140
python3 -m pip install -e .
```


# Camera Calibration

Having a calibrated camera is important for accurately converting measurements in images to real world measurements of distances and angles.  An OpenCV based camera calibration tool is included with sensors3140 and can be run using the following command.  

A printable calibration target can be found [here](extras/calibration_target.pdf).  After printing verify that the tags are 40mm from center to center.


```
python -m sensors3140.calibrate.calibrate_camera -o camera.json
```

It is a good idea to try calibrations for a variety of camera settings.  Running a camera at higher resolution may increase the detection distance may also slow down the frame rate and require more computation.  The tool has many configurable that can be shown with the `-h` option on the command line but not all configurations are supported for all cameras.  A higher resolution example is shown here:

```
python3 -m sensors3140.calibrate --width=1280 --height=720 --fps=15 -d --rotate=0 -o camera.json
```

To calibrate the camera move the apriltag calibration target in front of the camera **slowly** and **angling** it in many directions to get a variety of calibration images.  If the device has a monitor, the `-d` option will display a window that may help with this process and show apriltag detections.  Tilting the target in depth is important to allow algorithms to better estimate 3D parameters of the camera and better estimate depth.  The target should also be moved to cover the full extent of the camera field of view.

As the target is moved the tool will output camera calibration parameters to the terminal.  Continue to move the calibration target in front of the camera for a minute or two until the parameters converge.

When complete, a file called camera.json` will be created that includes camera options and calibration parameters.  This file will be loaded by other sensors3140 tools to configure the camera and compute accurate measurements.

# Apriltag

This simple python program is intended to be run on a coprocessor such as a Raspberry Pi 3/4 or NVIDIA Jetson Nano to locate apriltags in the field of view.  Apriltags are visual fiducial markers that can be easily and efficiently detected in images.  The markers may provide a variety of useful information such as the tag id, distance, angle, and pose of the marker relative to the camera.  These were invented at University of Michigan and [documentation, software, and publications](https://april.eecs.umich.edu/software/apriltag) can be found at their website.

Printable apriltag files can be found at https://github.com/rgov/apriltag-pdfs/tree/main/tag36h11.

The apriltag tool can be run from the command line and the `-h` option will display additional configuration options:

```
python3 -m sensors3140.apriltag
```

Apriltag detections are communicated through a NetworkTables interface in the table `apriltags`.

There is a `apriltag.json` file that configures the tool.  This should be updated with the proper ip for the network and the camera device.  This also references the `camera.json` file that is used to configure the camera settings.


## Tips

 * The speed or frame rate of the detections may depend on many things including the camera frame rate, processors speed, usb bus speed, etc.  
 * The robot motion may cause motion blur that make it difficult to accurately detect the edges of the apriltags.  Shortening the exposure time for the frames will reduce this issue if you have a camera that supports that option.
 * If running on a Romi the raspberry pi needs to be set to **Writable** in the web service before any changes can be made.


## Running as a service

In many cases you will want to boot your sensor and have the apriltag process run automatically.  There are many ways to accomplish this.  Here we install it as a unit in systemctl.  See these [examples](https://www.shellhacks.com/systemd-service-file-example/) for more a detailed tutorial or to customize the process.  Here we provide a tutorial and files needed for a typical raspberry pi coprocessor configuration.

The `sensors3140.service` file needs to be copied to `/etc/systemd/system/sensors3140.service` directory.

```
sudo cp config/sensors3140.service /etc/systemd/system/sensors3140.service
```

### Configuration Files

 * camera.json - A configuration file for the camera that includes the calibration parameters.  This should be created using the sensors3140 calibration tool.
 * apriltag.json - This controls basic parameters for the apriltag process.  It is important to verify the following:
    * networktables_host - should be set to the ip of your RoboRio. This is the primary method for communicating detections and your controller applications should read detection data from this table.  (For Romi this should be set to the ip of the development laptop.)
    * calibration - should specify the full path to a camera.json file.
    * tag_size - measured in millimeters, this this needs to be set properly to compute accurate distances and angles.  This is the measurement in mm for the outer black square.  160mm is the correct size for the sample files.  Apriltags for theFRC 2023 competition will be slightly larger at around 164.8mm.
    * sensor_id - controls the name of the subtable name in network tables.  By changing this it is possible to use multiple cameras and apriltag coprocessors on the same robot.

To install a basic configuration you can copy the json files to the following location.  This assumes the use of the preexisting raspberry pi v2.1 camera calibration file.  However, for most accurate results it is recommended that you generate the `camera.json` file yourself using the camera calibration tool.
```
cp config/apriltag.json /home/pi/apriltag.json
cp config/cameras/camera_rpi_21.json /home/pi/camera.json
```

Files to start the camera on boot can be installed like this:
```
sudo cp config/apriltag.service /etc/systemd/system/apriltag.service
sudo chmod 664 /etc/systemd/system/apriltag.service
```

# Running the AprilTag Field Map Visualizer

To run the AprilTag field map visualization tool that displays real-time tag detections and robot position:

```bash
python -m sensors3140.apriltag.maps.nt_map --server [NT_SERVER_IP] --game [GAME_ID]
```

## Parameters:

- `--server`: NetworkTables server IP address (default: 127.0.0.1)
- `--game`: Game field to load (default: 2025-reefscape)

## Example:

```bash
# Connect to local NetworkTables server
python -m sensors3140.apriltag.maps.nt_map

# Connect to robot at 10.31.40.2
python -m sensors3140.apriltag.maps.nt_map --server 10.31.40.2
```

The visualizer shows:
- Field map with AprilTag locations
- Currently detected tags (highlighted in green)
- Distance circles around detected tags
- Camera position and direction when tags are tracked
- Press 'q' to quit


### Editing the unitfile
The unit file sets the paths and configuration to start up the sensors3140 coprocessor.  It is a good idea to check the username, paths, etc to make sure the configuration is correct.

```
sudo nano /etc/systemd/system/sensors3140.service
```

### Testing the service
To load the changes into systemctl the daemon needs to be reloaded with the following command.

```
sudo systemctl daemon-reload
```

The system can be tested with the following commands:
```
sudo systemctl start sensors3140
sudo systemctl stop sensors3140
```

### Enable at boot
In order to start the service with the device boots run this command.
```
sudo systemctl enable sensors3140
```

### Automatically Restart the Service

https://ma.ttias.be/auto-restart-crashed-service-systemd/


### Checking the logs
If there are issues you should be able to check the log with:
```
journalctl -u sensors3140
```

Setup a git connection

```sh
git remote add steambot ssh://dev3140@raspberrypi.local/home/dev3140/git/sensors3140/
```