from networktables import NetworkTables
import time

# Initialize NetworkTables
NetworkTables.startServer()

print( NetworkTables.isServer() )

# Serve forever
count = 0
while True:
    time.sleep(1)
    # Create an entry
    count += 1
    table = NetworkTables.getTable( "test_table" )
    table.putNumber( "count", count )
    time.sleep(1)

    print( "Server running", count )



