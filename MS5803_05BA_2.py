# Distributed with a free-will license.
# Use it any way you want, profit or free, provided it fits in the licenses of its associated works.
# MS5803_05BA
# This code is designed to work with the MS5803_05BA_I2CS I2C Mini Module available from ControlEverything.com.
# https://www.controleverything.com/content/Temperature?sku=MS5803-05BA_I2CS#tabs-0-product_tabset-2

import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)
bus.write_byte(0x76, 0x1E)
firstRun = []
firstRun1 = []
firstRun2 = []
time.sleep(0.5)

def LPF(x):
    global prevX
    if not firstRun1:
        prevX = x
        firstRun1.append(1)
    
    alpha = 0.9
    xlpf = alpha*prevX + (1 - alpha)*x
    prevX = xlpf
    return xlpf

def depthValue():
    global pp
    global name
    if not firstRun:
        pp = 0
        name = fileName()
        firstRun.append(1)
    # MS5803_05BA address, 0x76(118)
    #		0x1E(30)	Reset command
    
    ptime = time.time()
    # Read 12 bytes of calibration data
    # Read pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA2, 2)
    C1 = data[0] * 256 + data[1]

    # Read pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA4, 2)
    C2 = data[0] * 256 + data[1]

    # Read temperature coefficient of pressure sensitivity
    data = bus.read_i2c_block_data(0x76, 0xA6, 2)
    C3 = data[0] * 256 + data[1]

    # Read temperature coefficient of pressure offset
    data = bus.read_i2c_block_data(0x76, 0xA8, 2)
    C4 = data[0] * 256 + data[1]

    # Read reference temperature
    data = bus.read_i2c_block_data(0x76, 0xAA, 2)
    C5 = data[0] * 256 + data[1]

    # Read temperature coefficient of the temperature
    data = bus.read_i2c_block_data(0x76, 0xAC, 2)
    C6 = data[0] * 256 + data[1]

    # MS5803_05BA address, 0x76(118)
    #		0x40(64)	Pressure conversion(OSR = 256) command
    bus.write_byte(0x76, 0x40)

    time.sleep(0.001)

    # Read digital pressure value
    # Read data back from 0x00(0), 3 bytes
    # D1 MSB2, D1 MSB1, D1 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D1 = value[0] * 65536.0000 + value[1] * 256.0000 + value[2]

    # MS5803_05BA address, 0x76(118)
    #		0x50(64)	Temperature conversion(OSR = 256) command
    bus.write_byte(0x76, 0x50)

    time.sleep(0.001)

    # Read digital temperature value
    # Read data back from 0x00(0), 3 bytes
    # D2 MSB2, D2 MSB1, D2 LSB
    value = bus.read_i2c_block_data(0x76, 0x00, 3)
    D2 = value[0] * 65536 + value[1] * 256 + value[2]

    dT = D2 - C5 * 256
    TEMP = 2000 + dT * C6 / 8388608
    OFF = C2 * 262144 + (C4 * dT) / 32
    SENS = C1 * 131072.0000 + (C3 * dT ) / 128.0000
    T2 = 0
    OFF2 = 0
    SENS2 = 0

    if TEMP > 2000:
        
        T2 = 0
        OFF2 = 0
        SENS2 = 0
    elif TEMP < 2000:
        T2 = 3 * (dT * dT) / 8589934592
        OFF2 = 3 * ((TEMP - 2000) * (TEMP - 2000)) / 8
        SENS2 = 7 * ((TEMP - 2000) * (TEMP - 2000)) / 8
        if TEMP < -1500:
            SENS2 = SENS2 + 3 * ((TEMP + 1500) * (TEMP +1500))

    TEMP = TEMP - T2
    OFF = OFF - OFF2
    SENS = SENS - SENS2
    pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0000) / 100.0000
    pressure = pressure*10.207371-9886.5000
    pressure = LPF(pressure)
    rate = (pressure-pp)/(time.time()-ptime)
    pp = pressure
    cTemp = TEMP / 100.0
    fTemp = cTemp * 1.8 + 32

    # Output data to screen
    TimeTaken = timeStamp()
    with open(name+ '.txt','a') as file:
        #file.write(str(TimeTaken)+','+str(heading[0])+','+str(heading[1])+','+str(ax)+','+str(ay)+','+str(s1)+','+str(s2)+'\n')
        file.write(str(TimeTaken)+','+str(pressure)+'\n')

    return pressure

def fileName():
    year   = time.localtime().tm_year
    month  = time.localtime().tm_mon
    day    = time.localtime().tm_mday
    hour   = time.localtime().tm_hour
    minute = time.localtime().tm_min
    second = time.localtime().tm_sec
    fileName  = 'MS5803'+'_'+str(year)+'_'+str(month)+'_'+str(day)+'_'+str(hour)+'_'+str(minute)+'_'+str(second)
    
    return fileName

def timeStamp():
    
    global timestamp
    if not firstRun2:
        timestamp = time.time()
        firstRun2.append(1)
        
    return (time.time()-timestamp)

while True:
    pressure = depthValue()
    

