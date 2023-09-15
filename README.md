# RPLidar Code

This script is a stand-alone version of the RPLidar Code. 

##Running Program on Start:
There are multiple ways to do this. The two easiest ways are: 
 * Setting up a service
 * Utilizing the Raspberry Pi's Crontab

#Setting Up Raspberry Pi Service: 
To set up a service to run on Raspberry Pi start, run the following lines below: 
```
sudo cp config/rplidar.service /etc/systemd/system/rplidar.service
sudo chmod 664 /etc/systemd/system/rplidar.service
```
#Setting Up Rasberry Pi Chrontab: 
```
crontab -e
```
Add "@reboot python3 /home/dev3140/RPLidar/__main__.py" to the end of the Crontab
