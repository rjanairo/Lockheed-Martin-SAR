import cv2 as cv
import numpy as np
import dronekit as dk
import dronekit_sitl
from pymavlink import mavutil
import time
from time import gmtime, strftime
import os

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


def goto_position_target_local_ned(north, east, down):
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

def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)

    fx, fy = flow[y, x].T

    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    for (x1, y1), (x2, y2) in lines:
        # if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 > 15:
        if ((x2 - x1) > 20 or (x2 - x1) < -20) and ((y2 - y1) > 20 or (y2 - y1) < -20):
            cv.circle(vis, (x1, y1), 15, (0, 0, 255), -1)
            cv.circle(vis, (x2, y2), 15, (0, 0, 255), -1)

    # cv.polylines(vis, lines, 0, (0, 255, 0))

    return vis

def sleepNrecord(seconds):
    i = 0.05
    while i <= seconds:
        ret, img = cam.read()
        out_original.write(img)
        out_opt_flow.write(img)
        time.sleep( 0.05 )
        i += 0.05


#END of definitions

connection_string = '/dev/ttyACM0'	#Establishing Connection With Flight Controller
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)

#RECORDING VIDEO SETUP
dir_original = 'ORIGINAL'
dir_opt_flow = 'OPT_FLOW'
time_stamp = strftime("%Y-%m-%d_%H:%M:%S", time.localtime(time.time()))
fourcc = cv.VideoWriter_fourcc(*'XVID')
#set file to write original camera input
out_original = cv.VideoWriter(os.path.join(dir_original,'original_'+time_stamp+'.avi'),fourcc, 8.0, (640,480)) 
#set file to write processed frames with optical flow
out_opt_flow = cv.VideoWriter(os.path.join(dir_opt_flow, 'opt_flow'+time_stamp+'.avi'),fourcc, 8.0, (640,480)) 

#Downloading Destination Coordinates from Flight Controller
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
waypoint2 = dk.LocationGlobalRelative(cmds[0].x, cmds[0].y, 3)  # Destination point 1
#waypoint2 = dk.LocationGlobalRelative(cmds[1].x, cmds[1].y, 3)  # Destination point 2

# START Flying
arm_and_takeoff(2)
vehicle.airspeed = 0.5 # set drone speed to be used with simple_goto
#vehicle.simple_goto(waypoint1)#trying to reach 1st waypoint
#time.sleep(20)

cam = cv.VideoCapture(0) # Get Initial Image from camera
ret, prev = cam.read()
h, w = prev.shape[:2]
prevgray = cv.cvtColor(prev, cv.COLOR_BGR2GRAY)

while True:	# Image processing/avoidance loop
    ret, img = cam.read()
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    flow = cv.calcOpticalFlowFarneback(prevgray, gray, None, 0.6, 5, 15, 3, 5, 1.2, 0)
    prevgray = gray
    new_frame = draw_flow(gray, flow)
    frame_HSV = cv.cvtColor(new_frame, cv.COLOR_BGR2HSV)  # convert to HSV
    frame_threshold = cv.inRange(frame_HSV, (0, 58, 140), (57, 255, 255))
    ret, thresh = cv.threshold(frame_threshold, 50, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #print ( len(contours) )
    areas = []
    centers = []
    for contour in contours:
        areas.append( cv.contourArea(contour) )

        x, y, w, h = cv.boundingRect(contour) 
        #centers.append( x + (w/2))
        centers.append( x )
        cv.rectangle(new_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if ( len(contours) ) >= 5 and max(areas) > 5000:
        print("turn right")
        send_ned_velocity(0, 0, 0)  # stop the vehicle for 2 seconds 
        sleepNrecord(2)
        goto_position_target_local_ned(0, 2, 0)  # move right for 2 metersq
        sleepNrecord(4)
    elif ( len(contours) ) >= 5 and max(areas) > 5000:
        print("turn left")
        send_ned_velocity(0, 0, 0)  # stop the vehicle for 2 seconds
        sleepNrecord(2)
        goto_position_target_local_ned(0, -2, 0)  # move left for 2 meters
        sleepNrecord(4)
    print("go to destination 2 sec") 
    vehicle.simple_goto(waypoint2)
    sleepNrecord(2)

    out_original.write(img)	#write original(unprocessed) image to the file
    out_opt_flow.write(new_frame)	#write image processed by Optical Flow to the file
    #cv.imshow("OpticalFlow", new_frame) 
    #cv.imshow("Original", frame_gray)
   
    key = cv.waitKey(30)
    if key == ord('q'):
        out_opt_flow.release()
        out_original.release()
        break	#END of Image Processing Loop (keyboard interrupt)
    lat = vehicle.location.global_relative_frame.lat  # get the current latitude
    lon = vehicle.location.global_relative_frame.lon  # get the current longitude
    if lat == cmds[0].x and lon == cmds[0].y:  
        print("Arrived")
        out_opt_flow.release()
        out_original.release()
        break	#END of Image Procwssing Loop (Arrival at Destination Point)

print("Landing")
#vehicle.mode = dk.VehicleMode("LAND")
#vehicle.flush()
