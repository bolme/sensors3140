

__version__ = '0.0.1'

import ntcore
import psutil
import socket
import time
import logging
import os

from .camera import Camera

# Config directory <home>/sensors3140/
# This is the directory where configuration files are stored.
sensors3140_directory = os.path.join(os.path.expanduser("~"), "sensors3140")
os.makedirs(sensors3140_directory, exist_ok=True)

# Set up logging to the sensor3140_directory
log_file = os.path.join(sensors3140_directory, "sensors3140.log")
logging.basicConfig(filename=log_file, level=logging.DEBUG)

# Print the time and date to the log file
logging.info("Starting sensors3140")
logging.info("Version: %s", __version__)
logging.info("Date: %s", time.strftime("%Y-%m-%d %H:%M:%S %Z"))


def connect_to_camera(camera_id):
    import cv2

    # load the camera configuration
    camera_config = load_camera_config(camera_id)
    if camera_config is None:
        print(f"Camera {camera_id} not configured.")
        return None
    
    # Create the camera object
    camera = cv2.VideoCapture(camera_id)
    if not camera.isOpened():
        print(f"Failed to open camera {camera_id}")
        return None
    
    # Set the camera parameters
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config['frame_size'][0])





def isValidInterfaceType(interface):
    types = ['en','wlan','eth','lo']
    for t in types:
        if interface.startswith(t):
            return True
    return False

def get_ip_addresses(include_localhost=False):
    ip_addresses = []
    
    for interface, addrs in psutil.net_if_addrs().items():
        if not isValidInterfaceType(interface):
            continue

        if not include_localhost and interface.startswith('lo'):
            continue
        
        for addr in addrs:
            if addr.family == socket.AF_INET:
                ip_addresses.append(addr.address)

    return ip_addresses

def get_hostname():
    return socket.gethostname()
    
def get_network_table_connection():
    from ntcore import NetworkTable


    inst = NetworkTable.getDefault()
    
    # Check if the connection is established
    if not inst.isConnected():
        print("Failed to connect to NetworkTables")
        return None

if __name__ == "__main__":
    import ntcore

    # Start a NetworkTables server
    server = ntcore.NetworkTableInstance.getDefault()
    server.startServer()

    time.sleep(1)

    inst = ntcore.NetworkTableInstance.getDefault()
    # start a NT4 client
    inst.startClient4("example client")
    # connect to a roboRIO with team number TEAM
    # connect to a specific host/port
    inst.setServer("127.0.0.1", ntcore.NetworkTableInstance.kDefaultPort4)

    inst.startDSClient()

    print("Client started")

    # Check if the connection is established
    if not inst.isConnected():
        print("Failed to connect to NetworkTables")
        exit(1)



