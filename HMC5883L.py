# HMC5888L Magnetometer (Digital Compass) wrapper class
# Based on https://bitbucket.org/thinkbowl/i2clibraries/src/14683feb0f96,
# but uses smbus rather than quick2wire and sets some different init
# params.

import smbus
import math
import time
import sys

class hmc5883l:

    __scales = {
        0.88: [0, 0.73],
        0.50729: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0,0)):
        self.bus = smbus.SMBus(port)
        self.address = address

        (degrees, minutes) = declination
        self.__declDegrees = degrees
        self.__declMinutes = minutes
        self.__declination = (degrees + minutes / 60) * math.pi / 180

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

    def declination(self):
        return (self.__declDegrees, self.__declMinutes)

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1<<len)
        return val

    def __convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def axes(self):
        data = self.bus.read_i2c_block_data(self.address, 0x00)
        x = self.__convert(data, 3)
        y = self.__convert(data, 7)
        z = self.__convert(data, 5)
        scale_x = 1.0455
        scale_y = 0.9006
        scale_z = 1.0716
        offset_x = -346.38
        offset_y = -189.98
        offset_z = -146.28
        xnew = (x-offset_x)*scale_x
        ynew = (y-offset_y)*scale_y
        znew = (z-offset_z)*scale_z

        heading = math.atan2(ynew, xnew)
        if heading < 0:
            heading += 2 * math.pi
        # if heading > 2 * math.pi:
            # heading -= 2 * math.pi
        heading = math.degrees(heading)
        
        roll = math.atan2(ynew, znew)
        if roll < 0:
            roll += 2 * math.pi
        roll = math.degrees(roll)

        pitch = math.atan2(znew, xnew)
        if pitch < 0:
            pitch += 2 * math.pi
        pitch = math.degrees(pitch)
                
        return (heading,roll,pitch)


# http://magnetic-declination.com/Great%20Britain%20(UK)/Harrogate#

##compass = hmc5883l(gauss = 0.50729, declination = (2,50))
##st = time.time()
##x,y,z = compass.axes()
##print(time.time()-st)
##while True:
##    x,y,z = compass.axes()
##    print(x,y,z)
##    
        
      
