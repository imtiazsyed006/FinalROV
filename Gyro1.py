
import smbus
import time

class L3G4200D(object):

    
    L3G4200D_ADDRESS = 0x69   
    address = L3G4200D_ADDRESS

    L3G4200D_REGISTER_WHO_AM_I = 0x0F
    L3G4200D_REGISTER_CTRL_REG1 = 0x20
    L3G4200D_REGISTER_CTRL_REG2 = 0x21
    L3G4200D_REGISTER_CTRL_REG3 = 0x22
    L3G4200D_REGISTER_CTRL_REG4 = 0x23
    L3G4200D_REGISTER_CTRL_REG5 = 0x24
    L3G4200D_REGISTER_OUT_X_L = 0x28
    L3G4200D_REGISTER_OUT_X_H = 0x29
    L3G4200D_REGISTER_OUT_Y_L = 0x2A
    L3G4200D_REGISTER_OUT_Y_H = 0x2B
    L3G4200D_REGISTER_OUT_Z_L = 0x2C
    L3G4200D_REGISTER_OUT_Z_H = 0x2D

    g = [0., 0., 0.]

    def __init__(self, debug=False, hires=False):


        self.bus = smbus.SMBus(1)  

        if self.bus.read_byte_data(self.address,
                self.L3G4200D_REGISTER_WHO_AM_I)&0xFF is not 0xD3:
            print( "error" )
        
        self.bus.write_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG1, 0xCF)
       
        self.bus.write_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG2, 0x01)
        
        self.bus.write_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG3, 0x08)
        
        self.bus.write_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG4, 0x90)
        
        self.bus.write_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG5, 0x02)


    def gyro16(self, high, low):
        n = (high << 8) | low   
        return n 

    def read(self):
        
        low = self.bus.read_byte_data(self.address,
          self.L3G4200D_REGISTER_OUT_X_L)
        high = self.bus.read_byte_data(self.address,
          self.L3G4200D_REGISTER_OUT_X_H)
        x = self.gyro16(high, low)
        low = self.bus.read_byte_data(self.address,
          self.L3G4200D_REGISTER_OUT_Y_L)
        high = self.bus.read_byte_data(self.address,
          self.L3G4200D_REGISTER_OUT_Y_H)
        y = self.gyro16(high, low)
        low = self.bus.read_byte_data(self.address,
          self.L3G4200D_REGISTER_OUT_Z_L)
        high = self.bus.read_byte_data(self.address,
          self.L3G4200D_REGISTER_OUT_Z_H)
        z = self.gyro16(high, low)
        if x & 0x8000: x -= 65536
        if y & 0x8000: y -= 65536
        if z & 0x8000: z -= 65536

        fs=self.bus.read_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG4)&0x30
        c1=self.bus.read_byte_data(self.address,
            self.L3G4200D_REGISTER_CTRL_REG1)

        
        if fs == 0x00: s=8.75
        elif fs == 0x10: s=17.5
        elif fs == 0x20: s=70
        elif fs == 0x30: s=70
        self.g[0] = float(x) * s / 1000.
        self.g[1] = float(y) * s / 1000.
        self.g[2] = float(z) * s / 1000.
        if self.g[0] > 0:
            self.g[0] = self.g[0] * (-1)
        elif self.g[0] < 0:
            self.g[0] = self.g[0] * (-1)
        return self.g[0],self.g[1],self.g[2]

##
##l3d4200d = L3G4200D()
##angle = 0
##while True:
##    startTime = time.time()
##    a,b,c = l3d4200d.read()
##    dt = time.time()-startTime
##    angle = angle + dt*a
##    with open('Data.txt','a') as file:
##        file.write(str(angle)+'\n')
##    
##
##l3d4200d.close()
