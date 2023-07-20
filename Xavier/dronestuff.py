import dronekit
try:
    vehicle = dronekit.connect("/dev/ttyACM0", baud=57600, wait_ready=True)
    pass

except dronekit.APIException:
    print("N")
    pass

except:
    print("No")
    pass
