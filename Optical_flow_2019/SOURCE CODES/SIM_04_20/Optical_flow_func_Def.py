import cv2 as cv
import numpy as np
import dronekit as dk
#import dronekit_sitl
from pymavlink import mavutil
import time
from time import gmtime, strftime
import os
import datetime

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
#================END OF DRONE FUNCTIONS DEFINITIONS =================================

def draw_flow_Areas(img, lines, threshold):
    
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    for (x1, y1), (x2, y2) in lines:
        #if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 > 35:
        #if abs(x1 - x2) > threshold or abs(y1 - y2) > threshold :
        if  abs(y1 - y2) > threshold :
            cv.circle(vis, (x1, y1), 15, (0, 0, 255), -1)
            cv.circle(vis, (x2, y2), 15, (0, 0, 255), -1)
    # cv.polylines(vis, lines, 0, (0, 255, 0))
    return  vis


def draw_flow(img, flow, step=8):
    h, w = img.shape[:2]
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)

    fx, fy = flow[y, x].T

    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    for (x1, y1), (x2, y2) in lines:
        #if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 > 35:
        if abs(x1 - x2) > 1 and abs(y1 - y2) > 1 :
            cv.circle(vis, (x1, y1), 15, (0, 0, 255), -1)
            cv.circle(vis, (x2, y2), 15, (0, 0, 255), -1)
    return vis

def get_grid_coords_w_flow (img, flow, VRange, HRange, step=16):
    h, w = img.shape[:2]
    #y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1).astype(int)
    y, x = np.mgrid[step/2 + h * VRange[0] : h * VRange[1] : step, step/2 + w*HRange[0] : w * HRange[1] : step].reshape(2,-1).astype(int)

    fx, fy = flow[y,x].T

    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    return lines

def get_amplitude_over_threshold(lines, dimension, thresholdX, thresholdY):
    new_lines = []

    for (x1, y1), (x2, y2) in lines:    
        if dimension == 'x':
            if thresholdX >= 0:
                if (x2 - x1) > thresholdX:
                    new_lines.append([[x1, y1], [x2, y2]])
            else:
                if (x2 - x1) < thresholdX:
                    new_lines.append([[x1, y1], [x2, y2]])

        elif dimension == 'y':
            if thresholdY >= 0:
                if abs((y2 - y1)) > thresholdY: #!!!!!!!!!!!!!!!!!EXPERIMENT!!!!!!!!!!!!!
                    new_lines.append([[x1, y1], [x2, y2]])
            else:
                if (y2 - y1) < thresholdY:
                    new_lines.append([[x1, y1], [x2, y2]])

        elif dimension == 'x_and_y':
            if thresholdX >= 0 and thresholdY > 0: #SHOULD BE MORE CONDITIONS!!!!!!!!!
                if (x2 - x1) > thresholdX or (y2 - y1) > thresholdY:
                    new_lines.append([[x1, y1], [x2, y2]])
            else:
                if (x2 - x1) < thresholdX and (y2 - y1) > thresholdY:
                    new_lines.append([[x1, y1], [x2, y2]])

        if dimension == 'xy': #not quite feneshed yet
            if thresholdX >= 0 : #just care for thresholdX
                if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 > max([abs(thresholdX), abs(thresholdY)]):
                    new_lines.append([[x1, y1], [x2, y2]])
            else:
                if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 < -(max([abs(thresholdX), abs(thresholdY)])):
                    new_lines.append([[x1, y1], [x2, y2]])
        
    return new_lines



def draw_simple_flow (img, lines):
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))

    for (x1, y1), (x2, y2) in lines:
         cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    
    return vis


def draw_simple_flow_over_threshold (img, lines):
   
    for (x1, y1), (x2, y2) in lines:
         cv.circle(img, (x1, y1), 5, (0, 0, 255), -1)

    return img

def get_decision_by_flow(new_lines):
    info = {}
    if len(new_lines)>0:

        for(x1, y1),(x2, y2) in new_lines:
        
            if  x1 < 320 : # HARDCODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                decision = "Right"
                info["decision"] = "Right"
                info["Coords"] = str([x1, y1]) 
                info["status"] = "Performing Maneuver" 
                return decision, info

            elif x1 > 320: # HARDCODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                decision = "Left"
                info["decision"] = "Left"
                info["Coords"] = str([x1, y1])
                info["status"] = "Performing Maneuver" 
                return decision, info

    else:
        decision =  "0"
        info["status"] = "Going to destination"

    return decision, info
    


def draw_flow_orig_parts(img, flow, part, parts, ldx, ldy,step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2 + h * (part-1)/parts: part*h/parts:step, step/2:w:step].reshape(2,-1).astype(int)

    fx, fy = flow[y,x].T
    print('Len of X:',len(x))
    
    print('Len of Y:',len(y))
    print('ldx:',len(ldx))
    print('ldy:',len(ldy))
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    cv.polylines(vis, lines, 0, (0, 255, 0))

    for (x1, y1), (x2, y2) in lines:
        cv.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
        

        if max(ldx)>5:
           if max(ldy)>5 or min(ldy)< 5 *(-1):
            cv.circle(vis, (x1,y1), 10, (255 , 0, 0), -1)
            print('X:'+str(max(ldx))+','+'Y:'+str(min(ldy)))
        
        if min(ldx)< 5 *(-1):
           if max(ldy)>5 or min(ldy)< 5 *(-1):
            cv.circle(vis, (x1,y1), 10, (255 , 0, 0), -1)
            print('X:'+str(max(ldx))+','+'Y:'+str(min(ldy)))


           
    return vis

def get_partial_flow(img, flow, part, parts, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step / 2 + h * (part - 1) / parts: part * h / parts:step, step / 2:w:step].reshape(2, -1).astype(int) # cuts the image horizontally
    #y, x = np.mgrid[step/2:h:step ,step/2+w*(part-1)/parts : part *h/parts:step].reshape(2,-1).astype(int) # cuts the image vertically
    ldx, ldy = flow[y, x].T

    return ldx,ldy # returns the flow of an image that was sliced horizontally


def make_decision_Areas_method(new_frame):
    frame_HSV = cv.cvtColor(new_frame, cv.COLOR_BGR2HSV)  # convert to HSV
    frame_threshold = cv.inRange(frame_HSV, (0, 58, 140), (57, 255, 255))
    ret, thresh = cv.threshold(frame_threshold, 50, 255, cv.THRESH_BINARY)
    contours, hiearchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    areas = []
    centers = []
    info = {}
    for contour in contours:
        areas.append( cv.contourArea(contour))

        x, y, w, h = cv.boundingRect(contour)  # then coordinate and height and width of the bounding box
        centers.append( x + (w/2))
        cv.rectangle(new_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

    if ( len(contours) ) >= 1 and max(areas) > 100:
        if np.median(centers) <= 320:
            decision = "Left"
            info["decision"] = "Left"
            info["Areas"] = str(max(areas))
            info["Centers"] = str(centers) 

        if np.median(centers) > 320:
            decision = "Right"
            info["decision"] = "Right"
            info["Areas"] = str(max(areas))
            info["Centers"] = str(centers) 

    else:
        decision = "0"

    return decision, info


def recording_setup_Windows(dir_original, dir_opt_flow):
    time_stamp = strftime("%Y-%m-%d_%H_%M_%S", time.localtime(time.time()))
    time_start = time.time()

    fourcc = cv.VideoWriter_fourcc(*'XVID')
    out_original = cv.VideoWriter(os.path.join(dir_original, 'original_' + time_stamp + '.avi'), fourcc, 8.0, (640, 480) )
    out_opt_flow = cv.VideoWriter(os.path.join(dir_opt_flow, 'opt_flow' + time_stamp + '.avi'), fourcc, 8.0,
    (640, 480))
    txt_file = open(os.path.join(dir_original, time_stamp + '.txt'), "w")
    return out_original, out_opt_flow, txt_file, time_start

def print_info(new_frame, info, txt_file, fr_count, time_start):
    y_adjust=0
    str_info=""
    time_end = time.time()
    for key in info:
        cv.putText(new_frame, key+':',
               org = (10,360+y_adjust),
               fontFace = cv.FONT_HERSHEY_COMPLEX,
               fontScale=1,
               color = (0,50,255),
               thickness = 1,
               lineType = cv.LINE_AA
               )
        

        cv.putText(new_frame, text=str(info[key]),
               org = (160, 360+y_adjust),
               fontFace = cv.FONT_HERSHEY_COMPLEX,
               fontScale = 1,
               color = (0, 50, 255),
               thickness = 1,
               lineType = cv.LINE_AA
               )


        y_adjust+=50
        print(fr_count, end = " ")
        print(key,str(info[key]), end = " ")
        str_info=str_info+info[key]+" "
        #file.write(str(fr_count) + " " + str(info[key]) + "\n")

    print()
    diff = time_end-time_start
    cv.putText(new_frame, str(fr_count),
               org=(10, 60),
               fontFace=cv.FONT_HERSHEY_COMPLEX,
               fontScale=1,
               color=(0, 50, 255),
               thickness=1,
               lineType=cv.LINE_AA
               )

    cv.putText(new_frame, "time: "+str(diff),
               org=(10, 100),
               fontFace=cv.FONT_HERSHEY_COMPLEX,
               fontScale=1,
               color=(0, 50, 255),
               thickness=1,
               lineType=cv.LINE_AA
               )

    txt_file.write(str(fr_count)+" "+str_info+str(diff)+"\n")

def sleepNrecord(seconds,cam, out_original, out_opt_flow, info, txt_file, fr_count, time_start):
    
    i=0.05
    while i <= seconds:
        fr_count += 1
        ret, img = cam.read()
        if ret:
            img = cv.resize(img, (640, 480))
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            print_info(gray, info, txt_file, fr_count, time_start)
            out_original.write(img)
            out_opt_flow.write(gray)
#================= THIS IS NOT IN THE DRONE VERSION ==================
            cv.imshow("OpticalFlow", gray)
#=====================================================================
        time.sleep(0.05)
        i+=0.05
    return out_original, out_opt_flow, fr_count, cam

def drone_Commands(center,area,boxes):
    print("drone commands")
    # if the contours are more than the set amount and
    # then make sure that you stop the drone and turn left

#END of definitions
