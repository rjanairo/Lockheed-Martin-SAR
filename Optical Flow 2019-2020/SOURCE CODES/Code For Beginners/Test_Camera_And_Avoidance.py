import cv2 as cv
import numpy as np
import dronekit as dk
import dronekit_sitl
from pymavlink import mavutil
import time
from time import gmtime, strftime
import os


def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step / 2:h:step, step / 2:w:step].reshape(2, -1).astype(int)

    fx, fy = flow[y, x].T

    lines = np.vstack([x, y, x + fx, y + fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
    for (x1, y1), (x2, y2) in lines:
        # if ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) ** 0.5 > 15:
        if ((x2 - x1) > 10 or (x2 - x1) < -10) and ((y2 - y1) > 10 or (y2 - y1) < -10):
            cv.circle(vis, (x1, y1), 15, (0, 0, 255), -1)
            cv.circle(vis, (x2, y2), 15, (0, 0, 255), -1)

    # cv.polylines(vis, lines, 0, (0, 255, 0))

    return vis
#END of definitions

cap = cv.VideoCapture(0)
#cap.set(3, 480)
#cap.set(4, 360)
#cap.set(5, 24)
ret, old_frame = cap.read()
old_gray = cv.cvtColor(old_frame, cv.COLOR_BGR2GRAY)#

#RECORDING VIDEO SETUP
dir_original = 'ORIGINAL'
dir_opt_flow = 'OPT_FLOW'
time_stamp = strftime("%Y-%m-%d_%H:%M:%S", time.localtime(time.time()))
fourcc = cv.VideoWriter_fourcc(*'XVID')
out_original = cv.VideoWriter(os.path.join(dir_original,'original_'+time_stamp+'.avi'),fourcc, 8.0, (640,480)) #set file to write original camera input
out_opt_flow = cv.VideoWriter(os.path.join(dir_opt_flow, 'opt_flow'+time_stamp+'.avi'),fourcc, 8.0, (640,480)) #set file to write processed frames with optical flow
#out = cv.VideoWriter('output2.avi',fourcc, 8.0, (640,480))#changed FPS from 20 to 8

#AVOIDANCE
while True:
    ret, frame = cap.read()
    if frame is None:
        break
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    flow = cv.calcOpticalFlowFarneback(old_gray, frame_gray, None, 0.5, 5, 15, 3, 5, 1.2, 0)
    new_frame = draw_flow(frame_gray, flow)
    frame_HSV = cv.cvtColor(new_frame, cv.COLOR_BGR2HSV)
    frame_threshold = cv.inRange(frame_HSV, (0, 58, 140), (57, 255, 255))

    ret, thresh = cv.threshold(frame_threshold, 50, 255, cv.THRESH_BINARY)

    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
        area = cv.contourArea(contours[i])
        if area > 5000 and area < 13000:
            contours[0] = contours[i]
            x, y, w, h = cv.boundingRect(contours[i])
            cv.rectangle(new_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)#Drawing Rectangles
            center_x = x + w / 2
            center_y = y + h / 2
            if (center_x < 160):
                if (x + w <= 160):  # when the obstacle is on the left side of the screen < 160
                    print("go straight")
                else:
                    print("turn right")  # when the obstacle is on the left side but it's > 160
                  
            elif (center_x > 160) and (center_x <= 320):
                print("turn right")  # the obstacle is on the center_left of the screen
                
            elif (center_x > 320) and (center_x < 480):  # the obstacle is on the center_right of the screen
                
                print("turn left")
                
            elif (center_x >= 480):  # the obstacle is on the right of the screen
                if (x < 480):
                    print("turn left")
                    
                else:
                    print("go straight")

    #save = cv.cvtColor(new_frame, cv.COLOR_GRAY2BGR)
    out_original.write(frame)
    out_opt_flow.write(new_frame)
    #out.write(new_frame)
    cv.imshow("OpticalFlow", new_frame) 
    #cv.imshow("Original", frame_gray)
    old_gray = frame_gray.copy()

    key = cv.waitKey(30)
    if key == ord('q'):
        out_opt_flow.release()
        out_original.release()
        #out.release()
        break

print("Landing")

