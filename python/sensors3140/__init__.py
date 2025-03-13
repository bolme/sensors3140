

__version__ = '0.0.1'

import ntcore
import psutil
import socket
import time
import logging
import os
import traceback
from sensors3140.camera import Camera
from sensors3140.tables.network_tables import NetworkTablesManager

# Config directory <home>/sensors3140/
# This is the directory where configuration files are stored.
sensors3140_directory = os.path.join(os.path.expanduser("~"), "sensors3140")
os.makedirs(sensors3140_directory, exist_ok=True)

# Set up logging to the sensor3140_directory
log_file = os.path.join(sensors3140_directory, "sensors3140.log")
logging.basicConfig(filename=log_file, level=logging.INFO)


logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

handler = logging.StreamHandler()
handler.setLevel(logger.level)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)

# Add the handler to the logger
logger.addHandler(handler)


# Print the time and date to the log file
logging.info("Starting sensors3140")
logging.info("Version: %s", __version__)
logging.info("Date: %s", time.strftime("%Y-%m-%d %H:%M:%S %Z"))



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
    
def publish_error_message(exc: Exception):
    """
    Publish an error message to NetworkTables.  Extra safety checking to avoid crashing due to reporting the error.
    """
    try:
        traceback.print_exc()
        tables = NetworkTablesManager()
        tables.setString("sensors3140/error_type", str(type(exc)))
        tables.setString("sensors3140/error_message", str(exc))
        tables.setDouble("sensors3140/error_timestamp", time.time())
        trace_back_string = traceback.format_exc()
        tables.setString("sensors3140/error_traceback", trace_back_string)
        error_count = tables.getDouble("sensors3140/error_count", 0)
        tables.setDouble("sensors3140/error_count", error_count + 1)
    except Exception as e:
        try:
            print(f"Failed to publish error message: {type(e)}: {e}")
        except:
            pass


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



