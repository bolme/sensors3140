

__version__ = '0.0.1'

import ntcore
import psutil
import socket

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
    hostname = get_hostname()
    addresses = get_ip_addresses()
    print("ip addresses:",addresses)
    
    ipaddr = "unknown"
    identity="sensors3140"
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient4(identity)
    #inst.setServer('127.0.0.1',0)

    if inst.isConnected():
        print("Connected to NetworkTables")
    else:
        print("Failed to connect to NetworkTables")

    sd = inst.getTable("SmartDashboard")
    sensor3140 = inst.getTable("sensors3140")
    info = sensor3140.getSubTable("service_info")
    info.putString("hostname",hostname)

    i = 0
    print("robotTime:", sd.getNumber("robotTime", -1))

if __name__ == "__main__":
    print(get_ip_addresses())
    print(get_hostname())
    get_network_table_connection()
