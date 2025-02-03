from networktables import NetworkTables
import time
import threading

# Function to notify when the connection is established
def connection_listener(connected, info):
    print(info, "; Connected=%s" % connected)
    with cond:
        notified[0] = True
        cond.notify()

# Initialize NetworkTables
NetworkTables.initialize("10.31.40.2")

# Add a connection listener
cond = threading.Condition()
notified = [False]
NetworkTables.addConnectionListener(connection_listener, immediateNotify=True)

# Wait for the connection to be established
with cond:
    print("Waiting for connection...")
    if not notified[0]:
        cond.wait()

print(NetworkTables.isConnected())

# Get the table
table = NetworkTables.getTable("test_table")

# Read the value
count = table.getNumber("count", -1)
print("Initial count", count)