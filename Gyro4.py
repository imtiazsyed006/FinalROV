import time
import smbus
import math
import Gyro1 as gyro
import Adxl345_1 as adxl


firstRun  = []
firstRun1 = []
firstRun2 = []
firstRun3 = []
def BodyToInertial(p, q, r, phi, theta):
    sinPhi   = math.sin(phi)
    cosPhi   = math.cos(phi)
    cosTheta = math.cos(theta)
    tanTheta = math.tan(theta)
    
    dotPhi   = p + q*sinPhi*tanTheta + r*cosPhi*tanTheta
    dotTheta = q*cosPhi              - r*sinPhi
    dotPsi   = q*sinPhi/cosTheta     + r*cosPhi/cosTheta
    
    return dotPhi, dotTheta, dotPsi

def HighPassFilter(xg,yg):
    global prevxg
    global prevyg
    global prevxHF
    global prevyHF
    if not firstRun2:
        prevxg = 0
        prevyg = 0
        prevxHF = 0
        prevyHF = 0
        firstRun2.append(1)
    alpha = 0.8798
    xghpf  = alpha*prevxHF + alpha*(xg - prevxg)
    yghpf  = alpha*prevyHF + alpha*(yg - prevyg)
    prevxHF = xghpf
    prevyHF = yghpf
    prevxg = xg
    prevyg = yg
    return xghpf,yghpf

def LowPassFilter(ax,ay):
    global prevaX
    global prevaY
    if not firstRun1:
        prevaX = ax
        prevaY = ay
        firstRun1.append(1)
    
    alpha = 0.8798
    axlpf = alpha*prevaX + (1 - alpha)*ax
    aylpf = alpha*prevaY + (1 - alpha)*ay
    prevaX = axlpf
    prevaY = aylpf
    return axlpf,aylpf

def BodyToInertial(p, q, r, phi, theta):
    sinPhi   = math.sin(phi)
    cosPhi   = math.cos(phi)
    cosTheta = math.cos(theta)
    tanTheta = math.tan(theta)
    
    dotPhi   = p + q*sinPhi*tanTheta + r*cosPhi*tanTheta
    dotTheta = q*cosPhi              - r*sinPhi
    dotPsi   = q*sinPhi/cosTheta     + r*cosPhi/cosTheta
    
    return dotPhi, dotTheta, dotPsi

def GetGyroValues():
    global prevPhi
    global prevTheta
    global prevPsi
    global phi
    global theta
    global gyroPhi
    global gyroTheta
    global name

    if not firstRun:
        prevPhi  = 0
        prevTheta = 0
        prePsi   = 0
        phi      = 0
        theta    = 0
        gyroPhi = 0
        gyroTheta = 0
        name = fileName()
        firstRun.append(1)
    TimeTaken = timeStamp()
    Previous_Time = time.time()
    a             = gyro.L3G4200D()
    xg,yg,zg      = a.read()
    a1,a2,a3,roll,pitch = adxl.adxlfor()
    Current_Time  = time.time()
    dt            = Current_Time-Previous_Time
    xg,yg,zg      = BodyToInertial(xg,yg,zg,prevPhi,prevTheta)
    gyroPhi       = gyroPhi+xg*dt
    gyroTheta     = gyroTheta+yg*dt
    ab, cd = HighPassFilter(gyroPhi,gyroTheta)
    roll,pitch = LowPassFilter(roll,pitch)
    roll = ab+roll
    pitch = cd+pitch
    prevPhi = roll
    prevTheta = pitch
    with open(name+ '.txt','a') as file:
         file.write(str(TimeTaken)+','+str(roll)+','+str(pitch)+','+str(a1)+','+str(a2)+','+str(a3)+'\n')    
    return roll,pitch

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

while True:
    GetGyroValues()
    


    
    
        
