import ntcore
import time

TEAM = 3140

def main():
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startClient3("3140")  # Replace with your team number

    print("Connected",inst.isConnected())

    table = inst.getTable("SmartDashboard")
    while True:
        value = table.getEntry("my_variable").getDouble(0)
        print("Value:", value)
        time.sleep(1)

if __name__ == "__main__":
    main()