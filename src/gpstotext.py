import dronekit as dk

from datetime import datetime
import time
import os


# INITIALIZING DRONE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
connection_string = '/dev/ttyACM0'	#Establishing Connection With PIXHAWK
vehicle = dk.connect(connection_string, wait_ready=True, baud=115200)# PIXHAWK is PLUGGED to NUC (RPi too?) VIA USB


dir_gps='/home/souyu/Desktop/FT_03_31'
now = datetime.now()
time_stamp = now.strftime("%Y-%m-%d_%H_%M_%S")

def gpsToText():
    time_milsec = str(round(time.time()*1000.0))
    # READ CURRENT COORDINATES FROM PIXHAWK-------------------
    lat = vehicle.location.global_relative_frame.lat  # get the current latitude
    lon = vehicle.location.global_relative_frame.lon  # get the current longitude
        #change to .txt for no mission planner
    out =  open(os.path.join(dir_gps, time_stamp + 'txt'), "a" )
    out.write(str(lat)+'\t'+str(lon)+'\t'+time_milsec+'\n')


    out.close()
    return None

gpsToText()
gpsToText()