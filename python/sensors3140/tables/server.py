# Run a local network tables server for testing purposes.

from networktables import NetworkTables
import time

def main():
    # Initialize NetworkTables
    NetworkTables.startServer()

    # Serve forever
    count = 0
    while True:
        time.sleep(1)
        # Create an entry
        count += 1
        table = NetworkTables.getTable("test_table")
        table.putNumber("count", count)
        time.sleep(1)

        print("Server running", count)

if __name__ == "__main__":
    main()
