import cv2
import numpy as np
import os
import time

def detect_red_circle():
    #cam = cv2.VideoCapture("DJI_0036")
    while True:
        #ret, frame = cam.read()
        img = cv2.imread("OpenCV_Logo_with_text.png")
        #img = cv2.GaussianBlur("OpenCV_Logo_with_text.png",(25,25), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_range = np.array([0,150,150], dtype = np.uint8)
        upper_range = np.array([180,255,255], dtype= np.uint8)

        new_img = cv2.inRange(hsv, lower_range, upper_range)
        new_img = cv2.medianBlur(new_img,5)

        circles = cv2.HoughCircles(new_img, cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)


        #circles = np.round(circles[0, :]).astype(int)
        circles = np.uint16(np.around(circles))
        print(len(circles))
        for i in circles[0,:]:
        # draw the circle in the output image, then draw a rectangle
        # corresponding to the center of the circle
           cv2.circle(new_img, (i[0], i[1]), i[2], (0, 255, 0), 4)
           print(i[0],i[1],i[2])
           cv2.circle(new_img, (i[0], i[1]), i[2], (0,0,255),3)

        cv2.imshow('IMAGE', new_img)



        if(cv2.waitKey(10) & 0xFF == ord('q')):
            break
detect_red_circle()
