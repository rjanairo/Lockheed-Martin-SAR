import cv2 as cv
import numpy as np
#import dronekit as dk
#import dronekit_sitl
#from pymavlink import mavutil
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

    # cv.polylines(vis, lines, 0, (0, 255, 0))

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
            print("turn left")
        if np.median(centers) > 320:
            decision = "right"
            print("turn right")
    else:
        decision = "0"

    return decision

# END of definitions


# RECORDING VIDEO SETUP
dir_original = 'ORIGINAL' #
dir_opt_flow = 'OPT_FLOW'

out_original, out_opt_flow = recording_setup_Windows(dir_original, dir_opt_flow)

cam = cv.VideoCapture(0)

ret, prev = cam.read() #Reads a frame from camera
h, w = prev.shape[:2]  # gets height and width of the image (in pixels)
prevgray = cv.cvtColor(prev, cv.COLOR_BGR2GRAY) # converts image to grayscale (needed for processing)

fr_count = 0
while True:
    fr_count += 1
    ret, img = cam.read(0) # reads NEXT frame from camera (because we need both prevoius and next to analyze flow)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    flow = cv.calcOpticalFlowFarneback(prevgray, gray, None, 0.6, 5, 15, 3, 5, 1.2, 0)
    #if fr_count == 1:
        #print(len(flow[0]))
    #flow is the array that contains x and y displacements of ALL the pixels. if no movements, displacements are 0
    # you can see that to get the flow, we need to pass 2 images - prevgray and gray
    prevgray = gray #now "current" image becomes "prevoius"
    new_frame = draw_flow(gray, flow) # this one just creates image with green dots on it, for illustration purposes

    make_decision_Areas_method(new_frame)

    # if max(ldx) > threshold: #because it's LEFT half of image, we should avoif if there is rightwardward movement (positive displacement)
    #     print("turn right")
    #     cv.putText(new_frame, "turn right!", (10, 50), cv.FONT_HERSHEY_PLAIN, 3, (255, 255, 255))
    #
    #
    # if min(ldx) < threshold * (-1):#because it's right half of image, we should avoif if there is leftward movement (negative displacement)
    #     print("turn left")
    #     cv.putText(new_frame, "turn left!", (400, 50), cv.FONT_HERSHEY_PLAIN, 3, (255, 255, 255))
#----------------------------------------------------------------------------------------------------
    out_original.write(img)
    out_opt_flow.write(new_frame)
    #out.write(new_frame)
    cv.imshow("OpticalFlow", new_frame) #displaying image with flow on it, for illustration purposes
    #cv.imshow("Original", img)
    #cv.imwrite('frame' + str(fr_count) + '.jpg', new_frame)

    key = cv.waitKey(30)
    if key == ord('q'):
        out_opt_flow.release()
        out_original.release()
        break

print("Landing")

