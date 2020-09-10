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

    if not firstRun:
        prevPhi  = 0
        prevTheta = 0
        prePsi   = 0
        phi      = 0
        theta    = 0
        gyroPhi = 0
        gyroTheta = 0
        firstRun.append(1)
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
    return a1,a2,a3,roll,pitch



    


    
    
        
