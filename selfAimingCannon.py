import cv2, sys, time, RPi.GPIO as GPIO, math
from pycomms import PyComms
from rrb2 import *
import servoLib, motorControlLib, cameraLib, mpu6050, mpu6050Filtered, angleCalc, userInterface, distanceCalc
pitch = 0
#startup
startup = False
while not startup:
    while time.time < 2:
        calBias()
    cameraSetUp()
    startup = True
    pitch = update(False)
    servoStart()

#main loop
stopped = True
while True:
    servoPos, turretError = updateCameras()
    pitch = updateMPU(stopped)
    setServo(servoPos)
    #distance calc lib need to write
    targetPitch = calcAnlge(calDistance(servoPos))
    canFire = motorControl(turretError, pitch, targetPitch)
    userInterface(canFire)
    
