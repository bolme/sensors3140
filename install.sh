#/bin/bash
#

echo Installing dependencies
sudo apt install cmake

echo Creating the virutal environment in vision
python3 -m venv /home/dev3140/vision
source /home/dev3140/vision/bin/activate

echo Installing python scripts
pip install -e .

echo Creating the service file
sudo cp config/sensors3140.service /etc/systemd/system/sensors3140.service

sleep 2

echo Enabling the service
sudo systemctl daemon-reload
sudo systemctl enable sensors3140

