import cv2
import sys
import time
import RPi.GPIO as GPIO
import math
from rrb2 import *
# const
RIGHT_SERVO_PIN = 19  # the pin for the right servo
LEFT_SERVO_PIN = 20  # the pin for the left servo

sampleTime = 100  # the time between pid update in millis
maxOutput = 1  # the max value for the pid
minOutput = -1  # the min value for the pid
reverse = 0  # the value for reverse
forward = 1  # the value for forward
Xkp = 1
Xki = 1
Xkd = 1
ykp = 1
yki = 1
ykd = 1
xSetPoint = 240

servoPos = 127
Frame_W = 420
Frame_H = 100
SERVO_CAM0 = cv2.VideoCapture(0)
SERVO_CAM1 = cv2.VideoCapture(1)
cascPath = '/usr/share/opencv/lbpcascades/lbpcascade_frontalface.xml'
faceCascade = cv2.CascadeClassifier(cascPath)

surfaceAreaOfBall = .8620530241
massOfBall = 0.0594
gravity = 9.8
acceleration = 50

notReadyLedPin = 13
readyLedPin = 16
fireBottonPin = 17
triggerPin = 18

distanceBetween = 10

ARDUINO_READY = 12
PITCH = 6
ROLL = 5
YAW = 4


# variables
xLastRun = 0
xLastInput = 0
xDirection = 1
xITerm = 0
xNoCorrection = False
xLastOut = 1
yITerm = 0
yLastInput = 0
yDirection = 1
yLastRun = 0
yNoCorrection = False
yLastOut = 1

rightServoPan = 127
leftServoPan = 127


def milis():  # Get the time in millis
    return time.time()*1000


def xIValue():
    global Xki
    Xki *= sampleTime / 1000


def XDValue():
    global Xkd
    Xkd /= sampleTime / 1000


def yIValue():
    global yki
    yki *= sampleTime / 1000


def yDValue():
    global ykd
    ykd /= sampleTime / 1000


def xPID(Input, setpoint):
    global xDirection
    global xNoCorrection
    global xLastRun
    global xLastInput
    global xITerm
    global xLastOut
    now = millis()-xlastRun
    if now - xLastRun >= sampleTime:

        # if ki or kd have change recompute them
        if Xki != Xki:
            xIValue()
        if Xkd != Xkd:
            xDValue()

        # compute all the working error variables
        error = float(setpoint - Input)
        xITerm = float(Xki * error)
        if error > maxOutput:
            xITerm = maxOutput
        elif error < minOutput:
            xITerm = minOutput
        dinput = Input - xLastInput

        # compute PID output
        output = Xkp * error + xITerm - Xkd * dinput
        if output > maxOutput:
            output = maxOutput
        elif output < minOutput:
            output = minOuput

        # check if no movement is need
        if output < .1:
            xNoCorrection = True
        else:
            xNoCorrection = False

        # remember some variables for next time
        xLastInput = Input
        xLastRun = now
        xLastOut = math.abs(output)

        # set motor direction

        if output < 0:
            xDirection = reverse
        else:
            xDirection = forward
        return math.abs(output)
    return xLastOut


def yPID(Input, SetPoint):
    global yITerm
    global yLastInput
    global yLastRun
    global yDirection
    global yNoCorrection
    global yLastOut
    now = millis()-ylastRun
    if now - yLastRun >= sampleTime:

        # if ki or kd have change recompute them
        if yki != yki:
            yIValue()
        if ykd != ykd:
            yDValue()

        # compute all the working error variables
        error = float(SetPoint - Input)
        if error < Input*.1:
            return 0
        yITerm = float(yki * error)
        if error > mayOutput:
            yITerm = mayOutput
        elif error < minOutput:
            yITerm = minOutput
        dinput = input - yLastInput

        # compute PID output
        output = ykp * error + yITerm - ykd * dinput
        if output > mayOutput:
            output = maxOutput
        elif output < minOutput:
            output = minOuput

        # check if no movement is need
        if output < .1:
            yNoCorrection = True
        else:
            yNoCorrection = False

        # remember some variables for next time
        yLastInput = Input
        yLastRun = now
        yLastOut = math.abs(output)

        # set motor direction
        if output < 0:
            yDirection = reverse
        else:
            yDirection = forward
        return math.abs(output)
    return yLastOut


def calcAngle(distance):
    inside_sin = (gravity * distance) / acceleration ** 2
    if math.abs(inside_sin) > 1:
        print ("Out of Range")
    return math.degrees(math.sin(inside_sin)) / 2


def distanceAndYawCal(angle1, angle2):
    angle1 = math.radians(angle1)
    angle2 = math.radians(angle2)
    distance = distanceBetween*((math.sin(angle1)*math.sin(angle2))/math.sin(angle1+angle2))
    yaw = 180 - (((180 - (math.degrees(angle1) + math.degrees(angle2)))/2) + math.degrees(angle1))
    return distance, yaw


# contorl function
def setServos(r, l):
    r_duty = float(r) / 10.0 + 2.5
    l_duty = float(l) / 10.0 + 2.5
    rServo.ChangeDutyCycle(r_duty)
    lServo.ChangeDutyCycle(l_duty)


def motorControl(xInput, xSetpoint, yInput, ySetpoint):
    rr.set_motors(xPID(xInput, xSetpoint), xDirection, yPID(yInput, ySetpoint), yDirection)
    if xNoCorrection and yNoCorrection:
        return True
    else:
        return False


def servoCamRight():
    # Capture frame-by-frame
    global rightServoPan
    ret, frame = SERVO_CAM0.read()

    if not ret:
        print("Error getting image")

    # convert to greyscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.qualizeHist(gray)

    # do face detection
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))

    for(x, y, w, h) in faces:
        # draw a green rectangle around the face
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # track face

        # get the center of the face
        x += (w/2)

        # correct relative to center of image
        turnX = float(x - (Frame_W/2))

        # convert to perventage offset
        turnX /= float(Frame_W/2)

        # scale offest to degrees
        turnX *= 2.5  # VFOV
        rightServoPan += -turnX

        # limit to 0 through 180 degrees
        rightServoPan = max(0, min(180, rightServoPan))

        break
    frame = cv2.resize(frame, (540, 300))
    frame = cv2.flip(frame, 1)

    # Display the image, with rectangle
    # on the Pi desktop
    cv2.imshow('Video', frame)

    return servoPan


def servoCamLeft():
    global leftServoPan
    # Capture frame-by-frame
    ret, frame = SERVO_CAM1.read()

    if not ret:
        print("Error getting image")

    # convert to greyscale for detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.qualizeHist(gray)

    # do face detection
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))

    for(x, y, w, h) in faces:
        # draw a green rectangle around the face
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # track face

        # get the center of the face
        x += (w/2)

        # correct relative to center of image
        turnX = float(x - (Frame_W/2))

        # convert to perventage offset
        turnX /= float(Frame_W/2)

        # scale offest to degrees
        turnX *= 2.5  # VFOV
        leftServoPan += -turnX

        # limit to 0 through 180 degrees
        leftServoPan = max(0, min(180, leftServoPan))

        break
    frame = cv2.resize(frame, (540, 300))
    frame = cv2.flip(frame, 1)

    # Display the image, with rectangle
    # on the Pi desktop
    cv2.imshow('Video', frame)

    return servoPan


def updateCameras():
    return servoCamLeft(), servoCamRight()


def userInterface(readToFire):
    if readToFire:
        GPIO.output(notReadyLedPin, GPIO.LOW)
        GPIO.output(readyLedPin, GPIO.HIGH)
        if GPIO.input(fireBottonPin):
            GPIO.output(triggerPin, GPIO.HIGH)
        else:
            GPIO.output(triggerPin, GPIO.LOW)
    else:
        GPIO.output(readLedPin, GPIO.LOW)
        GPIO.output(notReadyLedPin, GPIO.HIGH)
        GPIO.output(triggerPin, GPIO.LOW)


def setup():
    global rServo
    global lServo
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RIGHT_SERVO_PIN, GPIO.OUT)
    rServo = GPIO.PMW(RIGHT_SERVO_PIN, 100)
    rServo.start(5)
    lServo = GPIO.PMW(LEFT_SERVO_PIN, 100)
    lServo = GPIO.start(5)

    GPIO.setwarnings(False)
    GPIO.setup(notReadyLedPin, GPIO.OUT)
    GPIO.setup(readyLedPin, GPIO.OUT)
    GPIO.setup(triggerPin, GPIO.OUT)

    SERVO_CAM0.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, Frame_W)
    SERVO_CAM0.set(cv2.cv.CV_CAP - PROP_FRAME_HEIGHT, Frame_H)
    SERVO_CAM1.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, Frame_W)
    SERVO_CAM1.set(cv2.cv.CV_CAP - PROP_FRAME_HEIGHT, Frame_H)
    time.sleep(2)
    return True


def readArduino(pin):
    if pin == ARDUINO_READY:
        return GPIO.input(ARDUINO_READY)
    else:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        count = 0
        time.sleep(.1)

        GPIO.setup(pin, GPIO.IN)
        while GPIO.input(pin) == GPIO.LOW:
            count += 1
        if pin == YAW:
            return (count - 0) * (180 - (-180) / ((200 - 0) + -180))
        else:
            return (count - 0) * (90 - 0) / ((200 - 0) + 0)
# startup
setup()
lServo = GPIO.PMW(LEFT_SERVO_PIN, 100)
rServo = GPIO.PMW(RIGHT_SERVO_PIN, 100)
# main loop
while True:
    leftServo, leftServo = updateCameras()
    getPitch = readArduino(PITCH)
    getYaw = readArduino(YAW)
    setServo(servoPos)
    distance, angle = distanceAndYawCal(rightServo, leftServo)
    targetPitch = calcAnlge(distance)
    canFire = motorControl(getYaw, angle, getPitch, targetPitch)
    userInterface(canFire)
