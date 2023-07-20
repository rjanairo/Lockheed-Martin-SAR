#__all__ = ['arm_and_takeoff', 'Arming motors', 'Taking off!']

import dronekit as dk
from pymavlink import mavutil
import time
import math
import os

connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)# PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = dk.VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    time.sleep(8)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def goto_position_target_local_ned(north, east, down): #THIS FUNCTION IS NOT NEEDED IN THIS SCRIPT
#IT IS HERE FOR REFERENCE
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,  # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

# calculate the distance from waypoint
def distance_to_waypoint (clocation, nwaypoint): # arguments are current location and waypoint
    R = 6371000 #radius of Earth in meters
    x = math.pi*(nwaypoint[1]-clocation[1])/180*math.cos(math.pi/180*(clocation[0]+nwaypoint[0])/2)
    X = R*x
    y = math.pi/180*(nwaypoint[0]-clocation[0])
    Y = R*y
    return math.sqrt(X**2+Y**2)

# reads waypoints from mission planner
def readmission(aFileName):
    print ("Reading mission from file: %s\n" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(dir+'/'+aFileName) as f:
        for k, line in enumerate(f):
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                #ln_autocontinue=int(linearray[10].strip())
                cmd = dk.Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, 1,\
                                  ln_param1,ln_param2, ln_param3, ln_param4, ln_param5,\
                                  ln_param6, ln_param7) #ln_autocontinue,
                missionlist.append(cmd)
    return missionlist

def gpsToText(dir_gps,time_stamp):
    time_milsec = str(round(time.time()*1000.0))
    # READ CURRENT COORDINATES FROM PIXHAWK-------------------
    lat = vehicle.location.global_relative_frame.lat  # get the current latitude
    lon = vehicle.location.global_relative_frame.lon  # get the current longitude
        #change to .txt for no mission planner
    out =  open(os.path.join(dir_gps, time_stamp + 'txt'), "a" )
    out.write(str(lat)+'\t'+str(lon)+'\t'+time_milsec+'\n')


    out.close()
    return None