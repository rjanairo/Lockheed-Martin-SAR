import time
from xbee import XBee #RJ This file is all about the second drone collecting data from the first drone (Survey drone)
import serial
from digi.xbee.devices import XBeeDevice #RJ header needed for xbee library

PORT = "/dev/ttyUSB0" 

BAUDRATE = 9600 #RJ is the only given value default from the documentation

#open serial port
device = XBeeDevice(PORT, BAUDRATE) #RJ XBee device only dependant to the internet wifi
device.open() #RJ Starts xbee transmitter

altitude = 4

def send_UAV_data_Xbee(ICAO, pos_lat, pos_lon, pos_alt_rel,velocity,airspeed):
    
    #print("In send ADSB funtion\n")
    msg = "ICAO: " + ICAO + '\n'
    msg += "Lattitude: " + pos_lat + '\n'
    msg += "Longitude: " + pos_lon + '\n'
    msg += "Altitude: " + pos_alt_rel + '\n'
    msg += "Velocity: " + velocity + '\n'
    msg += "Airspeed: " + airspeed + '\n'

    return msg 

def recieve_GPS_coord_xbee(): #RJ this functions recieve data from the other drone that surveyed the area
    counter = 0
    coords = []

    print("Waiting for iteration  #...")
    while(counter <= 0):            #RJ While loop that waits for the data from the survey drone
        msg = device.read_data()
        print("Waiting...")
        time.sleep(1)
        
        if msg is not None:         #RJ If the data collected is ok
            counter = int(msg.data.decode())    
            print("Recieved coords!")
            print(counter)

    t_end = time.time() + 2 # 2 seconds

    while time.time() < t_end:      #RJ While loop that waits for the txt file from survey drone
        print("Waiting from GCS to confirm recieved...")
        device.send_data_broadcast("Size received")
    print("Continuing...")
    x = 1

    while(x!=counter+1):            #RJ collects txt file(s) and verifying if it has data or not
        msg = device.read_data()
        if msg is not None:
            count = int(msg.data.decode().split()[0])
            #print(count)
            if x == count:
                coords.append(msg.data.decode())
                print(coords[x-1])
                x+=1
    
    j = 0               #RJ while loop that shows the prints out loading status
    while(j != 5):
        print("Waiting from GCS to confirm finished...")
        device.send_data_broadcast("Finished")
        time.sleep(1)
        j += 1

    #change to .txt for no mission planner
    out1 =  open('/home/souyu/Desktop/FT_03_31/Spadra_Farm_Search_WPS.txt', "w" )
    for i in range(len(coords)):   #RJ not sure if the desktop txt file is from the survey drone
        print("writing coordinates...")
        lat= coords[i].split()[1]
        lon = coords[i].split()[2]
        out1.write(str(i+1)+'\t0\t3\t16\t0\t0\t0\t0\t'+str(lat)+'\t'+str(lon)
                +'\t'+str(altitude)+'\n')
    out1.close()
    return coords



#while True:


    #try:
        #print ("Receive data...")
        #device.send_data_broadcast(send_UAV_data_Xbee("A", "34.00", "-174.50", "30","10.0","5.0"))
        #device.send_data_broadcast("Com\n")
        #msg = device.read_data()
        #print(msg.data.decode())

coordinates = recieve_GPS_coord_xbee() #RJ idk why they leave this 

#print(coordinates)
time.sleep(1)

#    except KeyboardInterrupt:
#        break