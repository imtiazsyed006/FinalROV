
import smbus
import time
import math

bus = smbus.SMBus(1)

bus.write_byte_data(0x53, 0x2C, 0x0A)
bus.write_byte_data(0x53, 0x2D, 0x08)
bus.write_byte_data(0x53, 0x31, 0x08)
bus.write_byte_data(0x53, 0x1F, 2) #Y offset calibration
bus.write_byte_data(0x53, 0x20, 4) #Z offset calibration

time.sleep(0.5)
accX = []
accY = []
accZ = []

def adxlfor():
    data0 = bus.read_byte_data(0x53, 0x32)
    data1 = bus.read_byte_data(0x53, 0x33)

    xAccl = ((data1 & 0x03) * 256) + data0
    if xAccl > 511 :
        xAccl -= 1024

    data0 = bus.read_byte_data(0x53, 0x34)
    data1 = bus.read_byte_data(0x53, 0x35)

    yAccl = ((data1 & 0x03) * 256) + data0
    if yAccl > 511 :
        yAccl -= 1024

    data0 = bus.read_byte_data(0x53, 0x36)
    data1 = bus.read_byte_data(0x53, 0x37)

    zAccl = ((data1 & 0x03) * 256) + data0
    if zAccl > 511 :
        zAccl -= 1024

    xi = xAccl
    yi = yAccl
    zi = zAccl
    
    xi = xi/256
    yi = yi/256
    zi = zi/256
    
    roll  = math.degrees(math.atan2(yi,zi))
    pitch = math.degrees(math.atan2(xi,zi))
    
##    theta = math.degrees(math.asin(xi/9.8))
##    phi   = math.degrees(math.asin(-yi/(9.8*math.cos(theta))))
    
    ## the method shown in above two lines is mentiiioned in the book but it does
    ## not give accurate readings in terms of degrees. 

    return (9.8*xi,9.8*yi,9.8*zi,(roll),(pitch))
    #return(xAccl,yAccl,zAccl)



                                   



