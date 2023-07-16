import time
from xbee import XBee
from digi.xbee.devices import XBeeDevice

#open serial port
PORT = "/dev/ttyUSB0" 
BAUDRATE = 9600
device = XBeeDevice(PORT, BAUDRATE)
device.open()

def recieve_GPS_coord_xbee(mission_name):

    altitude = 8 # change altitude if needed
    counter = 0
    coords = []

    print("Waiting for iteration  #...")
    while(counter <= 0):
        msg = device.read_data()
        print("Waiting...")
        time.sleep(1)
        
        if msg is not None:
            counter = int(msg.data.decode())
            print("Recieved coords!")
            print(counter)

    t_end = time.time() + 2 # 2 seconds

    while time.time() < t_end:
        print("Waiting from GCS to confirm recieved...")
        device.send_data_broadcast("Size received")
    print("Continuing...")

    x = 1


    while(x!=counter+1):
        msg = device.read_data()
        if msg is not None:
            count = int(msg.data.decode().split()[0])
            #print(count)
            if x == count:
                coords.append(msg.data.decode())
                print(coords[x-1])
                x+=1
    j = 0
    while(j != 5):
        print("Waiting from GCS to confirm finished...")
        device.send_data_broadcast("Finished")
        time.sleep(1)
        j += 1

    #change to .txt for no mission planner
    out1 =  open(dir+'/'+mission_name, "w" )
    for i in range(len(coords)):
        print("writing coordinates...")
        lat= coords[i].split()[1]
        lon = coords[i].split()[2]
        out1.write(str(i+1)+'\t0\t3\t16\t0\t0\t0\t0\t'+str(lat)+'\t'+str(lon)
                +'\t'+str(altitude)+'\n')
    out1.close()
    return coords