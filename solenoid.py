import time
from pymavlink import mavutil

# Set the connection parameters (device, baudrate)
connection_string = '/dev/ttyACM0'
baud_rate = 115200

# Create a mavlink connection
master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
# check line 9
# Wait for the connection to be established
master.wait_heartbeat()

# Set the RELAY_PIN parameter to the pin number of the relay switch
RELAY_PIN = 54

# Set the maximum number of activations
MAX_ACTIVATIONS = 7

# Activate the relay switch every 5 seconds
for i in range(MAX_ACTIVATIONS):
    print(f"Activation {i+1}")
    
    # Set the relay switch to ON
    master.mav.command_long_send(
        master.target_system, 
        master.target_component, 
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 
        0,  # Unused parameter
        RELAY_PIN, 
        1,   # Turn the relay switch on
        0,   # Unused parameter
        0,   # Unused parameter
        0,   # Unused parameter
        0,   # Unused parameter
        0)   # Unused parameter

    # Wait for 1 second
    time.sleep(1)

    # Set the relay switch to OFF
    master.mav.command_long_send(
        master.target_system, 
        master.target_component, 
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 
        0,  # Unused parameter
        RELAY_PIN, 
        0,   # Turn the relay switch off
        0,   # Unused parameter
        0,   # Unused parameter
        0,   # Unused parameter
        0,   # Unused parameter
        0)   # Unused parameter

    # Wait for 5 seconds
    time.sleep(5)

# Close the mavlink connection
master.close()