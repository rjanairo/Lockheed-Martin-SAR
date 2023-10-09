#! /usr/bin/python

from lib_robotis import *
import cv2 #as cv
import numpy as np
import time

dyn = USB2Dynamixel_Device('/dev/ttyUSB0')
p = Robotis_Servo(dyn,1,series="MX")
move_to = float(sys.argv[1])/-30
print move_to
p.move_angle(math.radians(move_to))
#p.move_to_encoder(0)
#servo = p.read_angle()
#print servo
#time.sleep(1)
