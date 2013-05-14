import serial, time

def writeByteString (string):
    # can accept negative numbers and will convert them to twos complement. if the
    # number begins with x it is converted to a 16b number, otherwise 8b.
    def createBytesFromString (string):
        # takes a string representing a negative number and returns the two's complement
        # byte of that number (although it will be in the form of an unsigned byte).
        def convert8bUnsigned (unsigned_string):
            u_num = int(unsigned_string)
            byte = chr(u_num)
            return byte
        # NB: high byte first
        def convert16bUnsigned (unsigned_string):
            u_num = int(unsigned_string)
            hi_byte = chr(u_num >> 8)
            lo_byte = chr(u_num % (0xFF+1))
            return hi_byte + lo_byte
        def convert8bTwosComplement (negative_string):
            neg_num = int(negative_string)
            tc_num = 0xFF + 1 + neg_num
            byte = chr(tc_num)
            return byte
        # NB: high byte first
        def convert16bTwosComplement (negative_string):
            neg_num = int(negative_string)
            tc_num = 0xFFFF + 1 + neg_num
            hi_byte = chr(tc_num >> 8)
            lo_byte = chr(tc_num % (0xFF+1))
            return hi_byte + lo_byte
    
        r = ""
        string_splits = string.split()
        for i in range(len(string_splits)):
            if string_splits[i][0] == 'x':
                if string_splits[i][1] == '-':
                    r += convert16bTwosComplement(string_splits[i][1:])
                else:
                    r += convert16bUnsigned(string_splits[i][1:])
            else:
                if string_splits[i][0] == '-':
                    r += convert8bTwosComplement(string_splits[i])
                else:
                    r += convert8bUnsigned(string_splits[i])
        return r
    
    
    global port
    port.flushInput()
    port.write(createBytesFromString(string))


# note that the input should have been flushed prior to issuing the command to
# which this read is expecting a response.
def readByteString (length, signed=False):
    if length == 1:
        byte = ord(port.read())
        if signed and byte > 0x7F:
            return (0xFF+1) - byte
        else:
            return byte
    elif length == 2:
        hi_byte = ord(port.read())
        lo_byte = ord(port.read())
        if signed and hi_byte > 0x7F:
            if hi_byte > 0x7F:
                return (0xFFFF+1) - (((hi_byte & 0x7F) << 8) + lo_byte)
        else:
            return (hi_byte << 8) + lo_byte
    else:
        raise Exception("unsupported byte length given to readByteString")
    



def robotStart (mode_string=""):
    writeByteString("128")
    if mode_string == "safe" or mode_string == "full":
        robotSetMode(mode_string)

def robotSetMode (mode_string):
    if mode_string == "passive":
        writeByteString("128")
    elif mode_string == "safe":
        writeByteString("131")
    elif mode_string == "full":
        writeByteString("132")
    else:
        raise Exception("Unrecognised mode string")
    time.sleep(0.5)
    
# velocities are in mm/s
def robotSetSpeed (left_velocity, right_velocity):
    writeByteString("145 x%d x%d" % (left_velocity, right_velocity))

def robotSetDemo (demo_number):
    writeByteString("136 %d" % (demo_number))

def robotSetLEDs (playLED=False, advanceLED=False):
    led_bits = 0
    if playLED:
        led_bits |= 2
    if advanceLED:
        led_bits |= 8
    writeByteString("139 %d 0 255" % (led_bits))

# The distance that Create has traveled in millimeters since the distance it was
# last requested. This is the same as the sum of the distance traveled by both
# wheels divided by two. Positive values indicate travel in the forward
# direction; negative values indicate travel in the reverse direction. If the
# value is not polled frequently enough, it is capped at its minimum or maximum.
# Range: -32768 - 32767
def robotGetDistance ():
    writeByteString("142 19")
    return readByteString(2, signed=True)

# The angle in degrees that Create has turned since the angle was last requested.
# Counter-clockwise angles are positive and clockwise angles are negative. If the
# value is not polled frequently enough, it is capped at its minimum or maximum.  
# Range: -32768 - 32767
def robotGetAngle ():
    writeByteString("142 20")
    return readByteString(2, signed=True)

# returns a string describing the robot's current charging state
def robotGetCharging ():
    writeByteString("142 21")
    response = readByteString(1)
    try:
        return "charge status = " + robotGetCharging.responses[response]
    except:
        return "response = %d" % (response)
        pass
robotGetCharging.responses = {0: "not charging", 1: "reconditioning charging", 2: "full charging", 3: "trickle charging", 4: "waiting", 5: "charging fault condition"}

def robotGetTemp ():
    writeByteString("142 24")
    response = readByteString(1)
    return "temperature = %d'C" % (response)

def robotGetCurrent ():
    writeByteString("142 23")
    response = readByteString(2, signed=True)
    return "current = %d mA" % response

def robotGetVoltage ():
    writeByteString("142 22")
    response = readByteString(2, signed=False)
    return "voltage = %d mV" % response

def robotGetCharge ():
    writeByteString("142 25")
    response = readByteString(2, signed=False)
    return "charge = %d mAh" % response

def robotGetCapacity ():
    writeByteString("142 26")
    response = readByteString(2, signed=False)
    return "capacity = %d mAh" % response


# Initialise
print "Connecting to robot."
port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=2)
port.open()

print "Connection established, initialising."
#robotStart("safe")
robotStart()
time.sleep(0.20)

print "Initialised, reading sensors"
print robotGetTemp()
print robotGetCurrent()
print robotGetVoltage()
print robotGetCharge()
print robotGetCapacity()
print robotGetCharging()
"""
print "done, driving..."
print "distance = " + str(robotGetDistance())
robotSetSpeed(500, 500)
time.sleep(1)
robotSetSpeed(0, 0)
print "distance = " + str(robotGetDistance())

print "driven!"
"""
print "shutting down"
port.close()

print "Finished."
