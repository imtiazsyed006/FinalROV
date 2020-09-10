import time
import smbus
import math
import Gyro1 as gyro
import Adxl345_1 as adxl
gyroI2CAddr = 105
gyroRaw = [0,0,0]
gyroDPS = [0,0,0]
heading = [0,0,0]
gyroZeroRate = [0,0,0]
gyroThreshold = [0,0,0]
firstRun  = []
firstRun2 = []
firstRun3 = []
s = [0,0]
note = []

GYRO_SIGMA_MULTIPLE = 3

dpsPerDigit = 0.00875

    
def BodyToInertial(p, q, r, phi, theta):
    sinPhi   = math.sin(phi)
    cosPhi   = math.cos(phi)
    cosTheta = math.cos(theta)
    tanTheta = math.tan(theta)
    
    dotPhi   = p + q*sinPhi*tanTheta + r*cosPhi*tanTheta
    dotTheta = q*cosPhi              - r*sinPhi
    dotPsi   = q*sinPhi/cosTheta     + r*cosPhi/cosTheta
    
    return dotPhi, dotTheta, dotPsi

def getDeltaTMicros():
    global lastTime
    if not firstRun:
        firstRun.append(1)
        lastTime = 0
        
    currentTime = time.time()
    deltaT = currentTime - lastTime
    if deltaT < 0.0:
        deltaT = currentTime-lastTime
    lastTime = currentTime
    return deltaT

def updateHeadings():
    global s1
    global s2
    global s3
    global st
    global name
    if not note: # check if it is first time run. Just to initiate global variable
        note.append(1)
        s1 = 0
        s2 = 0
        s3 = 0
##        st = time.time()
        name = fileName()

    TimeTaken      = timeStamp()
    alpha = 0.2
    deltaT = getDeltaTMicros()
    ax, ay, az, roll,pitch        = adxl.adxlfor()
    
    
    k1 = (gyroDPS[0]*deltaT)*108  # gyroDPS has gyroscope dps reading
    k2 = (gyroDPS[1]*deltaT)*108  # gyroDPS has gyroscope dps reading
    k3 = (gyroDPS[2]*deltaT)*108  # gyroDPS has gyroscope dps reading
    
    s1 = s1 - k1 #catching gyroscope drift
    s2 = s2 - k2 #catching gyroscope drift
    s3 = s3 - k3 #catching gyroscope drift
    
    heading[0] = (1-alpha)*(heading[0] - k1) + alpha*roll  # why heading - k? replacing it with + will just switch value sign
    heading[1] = (1-alpha)*(heading[1] - k2) + alpha*pitch #+ alpha*ay # pitch
    heading[2] = (heading[2] - k3)                         # supposed to be comp filtered yaw if I attach a magnetometer
    with open(name+ '.txt','a') as file:
         file.write(str(TimeTaken)+','+str(heading[0])+','+str(heading[1])+','+str(ax)+','+str(ay)+','+str(az)+'\n')
         #file.write(str(TimeTaken)+','+str(heading[0])+','+str(heading[1])+'\n')

def updateGyroValues():
    
    global prevPhi
    global prevTheta
    global prevPsi

    if not firstRun2:
        prevPhi   = 0
        prevTheta = 0
        prevPsi   = 0
        
    deltaGyro = [0,0,0]
    a = gyro.L3G4200D()
    xk,yk,zk = a.read()
    dotPhi, dotTheta, dotPsi = BodyToInertial(xk,yk,zk,prevPhi,prevTheta)
    gyroRaw[0] = dotPhi
    gyroRaw[1] = dotTheta
    gyroRaw[2] = dotPsi
    for i in range(3):
        deltaGyro[i] = ((gyroRaw[i]-gyroZeroRate[i])) 
        if abs(deltaGyro[i]) < gyroThreshold[i]:
            deltaGyro[i] = 0
        gyroDPS[i] = dpsPerDigit*deltaGyro[i]

    prevPhi   = heading[0]
    prevTheta = heading[1]
    prevPsi   = heading[2]

def calibrateGyro():
    gyroSums = [0,0,0]
    gyroSigma = [0,0,0]
    NUM_GYRO_SMAPLES = 50
    for i in range(50):
        updateGyroValues()
        for i in range(3):
            gyroSums[i] += gyroRaw[i]
            gyroSigma[i] += gyroRaw[i]*gyroRaw[i]

    for i in range(3):
        averageRate = gyroSums[i]/50
        gyroZeroRate[i] = averageRate
        gyroThreshold[i] = math.sqrt(((gyroSigma[i]) / 50) - (averageRate * averageRate)) * GYRO_SIGMA_MULTIPLE

def testCalibration():
    calibrateGyro()
    for i in range(3):
        print(gyroZeroRate[i],gyroThreshold[i])

def printDPS():
    print(gyroDPS[0],gyroDPS[1],gyroDPS[2])

def printHeadings():
    return ((heading[0]),int(heading[1]),int(heading[2]),s[0],s[1])

def fileName():
    year   = time.localtime().tm_year
    month  = time.localtime().tm_mon
    day    = time.localtime().tm_mday
    hour   = time.localtime().tm_hour
    minute = time.localtime().tm_min
    second = time.localtime().tm_sec
    fileName  = 'IMU'+'_'+str(year)+'_'+str(month)+'_'+str(day)+'_'+str(hour)+'_'+str(minute)+'_'+str(second)
    
    return fileName

def timeStamp():
    global timestamp
    if not firstRun3:
        timestamp = time.time()
        firstRun3.append(1)
        
    return (time.time()-timestamp)


calibrateGyro()

while True:
    
    updateGyroValues()
    updateHeadings()
    
    


