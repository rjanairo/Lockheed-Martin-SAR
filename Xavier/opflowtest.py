import cv2 as cv
import numpy as np
#import dronekit as dk
#import dronekit_sitl
#from pymavlink import mavutil
import time
from time import gmtime, strftime
import os
import matplotlib.pyplot as plt

'''
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
'''

def draw_flow(img, flow, step=8):
    h, w = img.shape[:2]
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)

    fx, fy = flow[y, x].T

    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    for (x1, y1), (x2, y2) in lines:
        #if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 > 35:
        if abs(x1 - x2) > 10 and abs(y1 - y2) > 10 :
            cv.circle(vis, (x1, y1), 15, (0, 0, 255), -1)
            cv.circle(vis, (x2, y2), 15, (0, 0, 255), -1)

    #cv.polylines(vis, lines, 0, (0, 255, 0))

    return vis


def draw_flow_orig(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)

    fx, fy = flow[y,x].T

    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (_x2, _y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis


def recording_setup_Windows(dir_original, dir_opt_flow):
    time_stamp = strftime("%Y-%m-%d_%H_%M_%S", time.localtime(time.time()))

    #file_test = open(os.path.join(dir_original, 'original_' + time_stamp + '.avi'), 'w')
    #file_test.write("Hello")
    #file_test.close()
    fourcc = cv.VideoWriter_fourcc(*'XVID')
    out_original = cv.VideoWriter(os.path.join(dir_original, 'original_' + time_stamp + '.avi'), fourcc, 8.0, (640, 480))
    out_opt_flow = cv.VideoWriter(os.path.join(dir_opt_flow, 'opt_flow' + time_stamp + '.avi'), fourcc, 8.0,
    (640, 480))  # set file to write processed frames with optical flow
    # out_original = cv.VideoWriter('original_' + time_stamp + '.avi', fourcc, 8.0,(640, 480))
    # out_opt_flow = cv.VideoWriter( 'opt_flow_' + time_stamp + '.avi', fourcc, 8.0,(640, 480))
    # changed FPS from 20 to 8
    return out_original, out_opt_flow

def make_decision_Areas_method(new_frame):
    frame_HSV = cv.cvtColor(new_frame, cv.COLOR_BGR2HSV)  # convert to HSV
    frame_threshold = cv.inRange(frame_HSV, (0, 58, 140), (57, 255, 255))
    ret, thresh = cv.threshold(frame_threshold, 50, 255, cv.THRESH_BINARY)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    #print ( len(contours) )
    areas = []
    centers = []
    for contour in contours:
        areas.append( cv.contourArea(contour) )

        x, y, w, h = cv.boundingRect(contour)  # then coordinate and height and width of the bounding box
        centers.append( x + (w/2))
        cv.rectangle(new_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if ( len(contours) ) >= 5 and max(areas) > 5000:
        if np.median(centers) <= 320:
            decision = "left"
            
        if np.median(centers) > 320:
            decision = "right"
            
    else:
        decision = "0"

    return decision

# END of definitions


# RECORDING VIDEO SETUP
dir_original = 'ORIGINAL' #
dir_opt_flow = 'OPT_FLOW'

out_original, out_opt_flow = recording_setup_Windows(dir_original, dir_opt_flow)

cap = cv.VideoCapture(0)

ret, frame = cap.read()
 
if ret:
    # resize frame
    frame = cv.resize(previous_frame, (960, 540))
 
    # upload resized frame to GPU
    gpu_frame = cv.cuda_GpuMat()
    gpu_frame.upload(frame)
 
    # convert to gray
    previous_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
 
    # upload pre-processed frame to GPU
    gpu_previous = cv.cuda_GpuMat()
    gpu_previous.upload(previous_frame)
 
    # create gpu_hsv output for optical flow
    gpu_hsv = cv.cuda_GpuMat(gpu_frame.size(), cv.CV_32FC3)
    gpu_hsv_8u = cv.cuda_GpuMat(gpu_frame.size(), cv2.CV_8UC3)
 
    gpu_h = cv2.cuda_GpuMat(gpu_frame.size(), cv.CV_32FC1)
    gpu_s = cv2.cuda_GpuMat(gpu_frame.size(), cv.CV_32FC1)
    gpu_v = cv2.cuda_GpuMat(gpu_frame.size(), cv.CV_32FC1)
 
    # set saturation to 1
    gpu_s.upload(np.ones_like(previous_frame, np.float32))

while True:
 
    # capture frame-by-frame
    ret, frame = cap.read()
 
    # upload frame to GPU
    gpu_frame.upload(frame)
 
    # add elapsed iteration time
    timers["reading"].append(end_read_time - start_read_time)
 
    # if frame reading was not successful, break
    if not ret:
        break
 
    # start pre-process timer
    start_pre_time = time.time()
 
    # resize frame
    gpu_frame = cv.cuda.resize(gpu_frame, (960, 540))
 
    # convert to gray
    gpu_current = cv.cuda.cvtColor(gpu_frame, cv.COLOR_BGR2GRAY)

# create optical flow instance
gpu_flow = cv.cuda_FarnebackOpticalFlow.create(
    5, 0.5, False, 15, 3, 5, 1.2, 0,
)
# calculate optical flow
gpu_flow = cv.cuda_FarnebackOpticalFlow.calc(
    gpu_flow, gpu_previous, gpu_current, None,
)

gpu_flow_x = cv.cuda_GpuMat(gpu_flow.size(), cv.CV_32FC1)
gpu_flow_y = cv.cuda_GpuMat(gpu_flow.size(), cv.CV_32FC1)
cv.cuda.split(gpu_flow, [gpu_flow_x, gpu_flow_y])
 
# convert from cartesian to polar coordinates to get magnitude and angle
gpu_magnitude, gpu_angle = cv.cuda.cartToPolar(
    gpu_flow_x, gpu_flow_y, angleInDegrees=True,
)
 
# set value to normalized magnitude from 0 to 1
gpu_v = cv.cuda.normalize(gpu_magnitude, 0.0, 1.0, cv.NORM_MINMAX, -1)
 
# get angle of optical flow
angle = gpu_angle.download()
angle *= (1 / 360.0) * (180 / 255.0)
 
# set hue according to the angle of optical flow
gpu_h.upload(angle)
 
# merge h,s,v channels
cv.cuda.merge([gpu_h, gpu_s, gpu_v], gpu_hsv)
 
# multiply each pixel value to 255
gpu_hsv.convertTo(cv.CV_8U, 255.0, gpu_hsv_8u, 0.0)
 
# convert hsv to bgr
gpu_bgr = cv.cuda.cvtColor(gpu_hsv_8u, cv.COLOR_HSV2BGR)
 
# send original frame from GPU back to CPU
frame = gpu_frame.download()
 
# send result from GPU back to CPU
bgr = gpu_bgr.download()
 

print("Landing")
