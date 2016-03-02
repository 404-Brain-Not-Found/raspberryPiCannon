import cv2, sys, time, RPi.GPIO as GPIO, math
from rrb2 import *
import servoLib, motorControlLib, cameraLib, arduinoCon, angleCalc, userInterface, distanceCalc
pitch = 0
#startup
startup = False
while not startup:
    arduinoSetup()
    cameraSetUp()
    servoStart()
    startup = True
    break

#main loop
stopped = True
while True:
    servoPos, turretError = updateCameras()
    pitch = getGyroValue()
    setServo(servoPos)
    targetPitch = calcAnlge(calDistance(servoPos))
    canFire = motorControl(turretError, pitch, targetPitch)
    userInterface(canFire)
    break

