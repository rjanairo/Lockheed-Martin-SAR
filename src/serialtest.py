import serial
import time

# allows reading the serial fully and avoid missing any bytes. *** MUST USE \n AT THE END OF LINE ***
def serialread():
    t_end = time.time() + 4             #set timer of 4 seconds
    time.sleep(.001)                    # delay of 1ms
    val = ser.readline()                # read complete line from serial output
    while not '\\n'in str(val):         # check if full data is received. 
        # This loop is entered only if serial read value doesn't contain \n
        # which indicates end of a sentence. 
        # str(val) - val is byte where string operation to check `\\n` 
        # can't be performed
        time.sleep(.001)                # delay of 1ms 
        temp = ser.readline()           # check for serial output.
        if not not temp.decode():       # if temp is not empty.
            val = (val.decode()+temp.decode()).encode("ascii", "ignore")
            # requrired to decode, sum, then encode because
            # long values might require multiple passes
        elif time.time() < t_end:       # break loop if alloted time for serial reading is passed.
            break
    val = val.decode('latin-1')                  # decoding from bytes
    val = val.strip()                   # stripping leading and trailing spaces.
    return val

#setting up xbee communication
#GPIO.setwarnings(False)
ser = serial.Serial(
    
    port='/dev/ttyUSB0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1   
)

while(True):
    print("Listening")
    read = serialread()
    print(read)