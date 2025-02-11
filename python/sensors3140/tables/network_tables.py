from networktables import NetworkTables
import threading
import logging
import time
import socket
import numpy as np

    
class NetworkTablesManager:

    _instance = None
    
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self, robot_ip_or_team: str = None):
        if not hasattr(self, 'initialized'):
            self.initialized = True
            self.robot_ip = robot_ip_or_team

            self.connected = False
            self.running = True
            
            # Setup logging
            self.logger = logging.getLogger('NetworkTables')
            self.logger.setLevel(logging.INFO)
            
            # Initial connection
            self._connect()
            
            # Start monitoring thread
            self.monitor_thread = threading.Thread(target=self._monitor_connection, daemon=True)
            self.monitor_thread.start()

    def _guess_robot_ip(self):
        "Try a few things to determine the robot IP address"
        my_ip_address = socket.gethostbyname(socket.gethostname())

    def _connect(self):
        try:
            NetworkTables.initialize(self.robot_ip)
            self.logger.info(f"Attempting connection to {self.robot_ip}")
        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
    
    def _monitor_connection(self):
        while self.running:
            current_connected = NetworkTables.isConnected()
            
            if current_connected != self.connected:
                self.connected = current_connected
                if current_connected:
                    self.logger.info("Connected to NetworkTables")
                else:
                    self.logger.warning("Lost connection to NetworkTables")
            
            # If disconnected, try to reconnect every 2 seconds
            if not self.connected:
                self._connect()
                
            time.sleep(2.0)
    
    def is_connected(self) -> bool:
        return self.connected
    
    def shutdown(self):
        self.running = False
        self.monitor_thread.join()

    def getDouble(self, path, default=float("nan")):
        if not self.connected:
            return float("nan")
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        return table.getEntry(path[-1]).getDouble(default)
    
    def setDouble(self, path, value):
        if not self.connected:
            return
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        table.getEntry(path[-1]).setDouble(value)

    def setString(self, path, value):
        if not self.connected:
            return
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        table.getEntry(path[-1]).setString(value)


    def getString(self, path, default=""):
        if not self.connected:
            return ""
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        return table.getEntry(path[-1]).getString(default)
    
    def getInteger(self, path, default=-1):
        if not self.connected:
            return -1
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        value = table.getEntry(path[-1]).getDouble(default)
        return int(value+0.5) # Round to nearest integer
    
    def setInteger(self, path, value):
        if not self.connected:
            return
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        table.getEntry(path[-1]).setDouble(value)



    def setDoubleArray(self, path, value):
        if not self.connected:
            return
        path = path.split('/')
        table = NetworkTables.getTable(path[0])
        for i in range(1,len(path)-1):
            table = table.getSubTable(path[i])
        table.getEntry(path[-1]).setDoubleArray(value)
                                                

    def get_robot_time(self) -> float:
        """Get robot match time in seconds"""
        if self.connected:
            try:
                table = NetworkTables.getTable("FMSInfo")
                return table.getNumber("MatchTime", -1.0)
            except:
                return -1.0
        return -1.0

def main():
    nt = NetworkTablesManager("127.0.0.1")
    try:
        while True:
            connected = nt.is_connected()
            robot_time = nt.get_robot_time() if connected else "N/A"
            arm_angle = nt.getDouble("Arm/CurrentAngle")
            print(f"Connected: {connected}, Robot Time: {robot_time}, Arm Angle: {arm_angle}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        nt.shutdown()
    except Exception as e:
        print(f"Error: {e}")
        nt.shutdown()

if __name__ == "__main__":
    main()