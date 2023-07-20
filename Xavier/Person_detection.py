#!/usr/bin/env python3

import fcntl, os
import time
import sys
import jetson.inference
import jetson.utils
from datetime import datetime


'''------------initialize jetson------------'''
net = jetson.inference.detectNet()    # start default detection network
video_input = jetson.utils.videoSource("/dev/video0", ["--input-width=1920", "--input-height=1080","--input-rate=21.0"]) # "/dev/video0" "flip-method=2" "--input-flip=rotate-180" 
# captures a camera (index 0). Change the resolution to tune the detection algorithm. 
# 4k = 3264 x 2464
# 720p = 1280 x 720
# 1080p = 1920 x 1080

# output = jetson.utils.videoOutput("rtp://192.168.1.129:1234") # use for remote viewing gstreamer

#record to avi
#'''
dir_original ='./'		# change this to folder where you want to store the video
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")
output = jetson.utils.videoOutput(os.path.join(dir_original, time_stamp + '.avi'))
#'''

print("Image Detection has started!" + str(time.time()))

confidence = 0
'''------------jetson initialized------------'''

max_confidence_normalized = 0.0    # best detection value normalized

while True:

    img = video_input.Capture()    # get an image from a camera
    # print(img.width, img.height)  # images height and width

    detections = net.Detect(img, overlay="box,labels,conf")    # gets a list of all detected objects
    if len(detections) > 0:
        people_confidence_list = []
        for detection in detections:    # go through all detections
            counter = net.GetClassDesc(detection.ClassID)
            if counter == 'person':    # If we see a person then:
                confidence = detection.Confidence    # get confidence value of a detection
                people_confidence_list.append(confidence)    # add the box to the list of all detected persons in an image
                # print("Center is at ", str(detection.Center))    # center of a detection box
                print("PErson detected")
                # print("Image variable list: \n", str(img))
        if len(people_confidence_list) > 0:
            max_confidence_normalized = 0.95 * max_confidence_normalized + 0.05 * max(people_confidence_list)   # update best detection normalized
        print("max confidence:",  max_confidence_normalized)
    # jetson.utils.saveImageRGBA('test.jpg',img,1280,720)    # save an image as a test.jpg
    jetson.utils.cudaDeviceSynchronize() # Allows to take both video and photo at same time
    output.Render(img)    # output image into an .avi file
"""
 <cudaImage object>
   -- ptr:      0x1013a6000
   -- size:     2764800
   -- width:    1280
   -- height:   720
   -- channels: 3
   -- format:   rgb8
   -- timestamp: 31.574766
   -- mapped:   true
   -- freeOnDelete: false
"""

    # time.sleep(0.1)			# might be the reason for lag
    
