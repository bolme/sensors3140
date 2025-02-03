import ntcore

if __name__ == "__main__":
    inst = ntcore.NetworkTableInstance.getDefault()
    inst.startServer()

    print("Server started")
    while True:
        pass