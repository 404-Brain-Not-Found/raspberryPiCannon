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
fireButtonPin = 17
triggerPin = 18

distanceBetween = 10

CON_PIN = 12
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


def _millis_():  # Get the time in millis
    return time.time()*1000


def _x_i_value_():
    global Xki
    Xki *= sampleTime / 1000


def _x_d_value_():
    global Xkd
    Xkd /= sampleTime / 1000


def _y_i_value_():
    global yki
    yki *= sampleTime / 1000


def _y_d_value_():
    global ykd
    ykd /= sampleTime / 1000


def _x_pid_(in_put, set_point):
    global xDirection
    global xNoCorrection
    global xLastRun
    global xLastInput
    global xITerm
    global xLastOut
    now = _millis_()
    if now - xLastRun >= sampleTime:

        # if ki or kd have change recompute them
        if Xki != Xki:
            _x_i_value_()
        if Xkd != Xkd:
            _x_d_value_()

        # compute all the working error variables
        error = float(set_point - in_put)/100
        xITerm = float(Xki * error)
        if error > maxOutput:
            xITerm = maxOutput
        elif error < minOutput:
            xITerm = minOutput
        d_input = in_put - xLastInput

        # compute PID output
        output = Xkp * error + xITerm - Xkd * d_input
        if output > maxOutput:
            output = maxOutput
        elif output < minOutput:
            output = minOutput

        # check if no movement is need
        if output < .1:
            xNoCorrection = True
        else:
            xNoCorrection = False

        # remember some variables for next time
        xLastInput = in_put
        xLastRun = now
        xLastOut = math.fabs(output)

        # set motor direction

        if output < 0:
            xDirection = reverse
        else:
            xDirection = forward
        return math.fabs(output)
    return xLastOut


def _y_pid_(in_put, set_point):
    global yITerm
    global yLastInput
    global yLastRun
    global yDirection
    global yNoCorrection
    global yLastOut
    now = _millis_()
    if now - yLastRun >= sampleTime:

        # if ki or kd have change recompute them
        if yki != yki:
            _y_i_value_()
        if ykd != ykd:
            _y_d_value_()

        # compute all the working error variables
        error = float(set_point - in_put)/100
        if error < in_put*.1:
            return 0
        yITerm = float(yki * error)
        if error > maxOutput:
            yITerm = maxOutput
        elif error < minOutput:
            yITerm = minOutput
        d_input = in_put - yLastInput

        # compute PID output
        output = ykp * error + yITerm - ykd * d_input
        if output > maxOutput:
            output = maxOutput
        elif output < minOutput:
            output = minOutput

        # check if no movement is need
        if output < .1:
            yNoCorrection = True
        else:
            yNoCorrection = False

        # remember some variables for next time
        yLastInput = in_put
        yLastRun = now
        yLastOut = math.fabs(output)

        # set motor direction
        if output < 0:
            yDirection = reverse
        else:
            yDirection = forward
        return math.fabs(output)
    return yLastOut


def _calc_angle_(_distance):
    inside_sin = (gravity * _distance) / acceleration ** 2
    if math.fabs(inside_sin) > 1:
        print ("Out of Range")
    return math.degrees(math.sin(inside_sin)) / 2


def _distance_and_yaw_cal_(angle1, angle2):
    angle1 = math.radians(angle1)
    angle2 = math.radians(angle2)
    _distance = distanceBetween*((math.sin(angle1)*math.sin(angle2))/math.sin(angle1+angle2))
    yaw = 180 - (((180 - (math.degrees(angle1) + math.degrees(angle2)))/2) + math.degrees(angle1))
    yaw = math.floor(yaw + .5)
    return _distance, yaw


# control function
def _set_servos_(r, l):
    r_duty = float(r) / 10.0 + 2.5
    l_duty = float(l) / 10.0 + 2.5
    rServo.ChangeDutyCycle(r_duty)
    lServo.ChangeDutyCycle(l_duty)


def _motor_control_(x_input, x_set_point, y_input, y_set_point):
    rr.set_motors(_x_pid_(x_input, x_set_point), xDirection, _y_pid_(y_input, y_set_point), yDirection)
    if xNoCorrection and yNoCorrection:
        return True
    else:
        return False


def _servo_cam_right_():
    # Capture frame-by-frame
    global rightServoPan
    ret, frame = SERVO_CAM0.read()

    if not ret:
        print("Error getting image")

    # convert to greyscale for detection
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray = cv2.qualizeHist(gray)

    # do face detection
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))

    for(x, y, w, h) in faces:
        # draw a green rectangle around the face
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # track face

        # get the center of the face
        x += (w/2)

        # correct relative to center of image
        turn_x = float(x - (Frame_W/2))

        # convert to perventage offset
        turn_x /= float(Frame_W/2)

        # scale offest to degrees
        turn_x *= 2.5  # VFOV
        rightServoPan += -turn_x

        # limit to 0 through 180 degrees
        rightServoPan = max(0, min(180, rightServoPan))

        break
    frame = cv2.resize(frame, (540, 300))
    frame = cv2.flip(frame, 1)

    # Display the image, with rectangle
    # on the Pi desktop
    cv2.imshow('Video', frame)

    return rightServoPan


def _servo_cam_left_():
    global leftServoPan
    # Capture frame-by-frame
    ret, frame = SERVO_CAM1.read()

    if not ret:
        print("Error getting image")

    # convert to greyscale for detection
    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray = cv2.qualizeHist(gray)

    # do face detection
    faces = faceCascade.detectMultiScale(frame, 1.1, 3, 0, (10, 10))

    for(x, y, w, h) in faces:
        # draw a green rectangle around the face
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # track face

        # get the center of the face
        x += (w/2)

        # correct relative to center of image
        turn_x = float(x - (Frame_W/2))

        # convert to perventage offset
        turn_x /= float(Frame_W/2)

        # scale offest to degrees
        turn_x *= 2.5  # VFOV
        leftServoPan += -turn_x

        # limit to 0 through 180 degrees
        leftServoPan = max(0, min(180, leftServoPan))

        break
    frame = cv2.resize(frame, (540, 300))
    frame = cv2.flip(frame, 1)

    # Display the image, with rectangle
    # on the Pi desktop
    cv2.imshow('Video', frame)

    return leftServoPan


def _offset_calibrate_():
    while _read_arduino_(CON_PIN) == GPIO.LOW:
        time.sleep(.01)
    time.sleep(1)
    done = False
    while not done:
        if _read_arduino_(CON_PIN) == GPIO.LOW:
            rr.set_motors(1, reverse, 0, reverse)
        else:
            rr.set_motors(0, reverse, 0, reverse)
            done = True


def _update_cameras_():
    return _servo_cam_left_(), _servo_cam_right_()


def _user_inter_face_(read_to_fire):
    if read_to_fire:
        GPIO.output(notReadyLedPin, GPIO.LOW)
        GPIO.output(readyLedPin, GPIO.HIGH)
        if GPIO.input(fireButtonPin) == GPIO.HIGH:
            GPIO.output(triggerPin, GPIO.HIGH)
        else:
            GPIO.output(triggerPin, GPIO.LOW)
    else:
        GPIO.output(readyLedPin, GPIO.LOW)
        GPIO.output(notReadyLedPin, GPIO.HIGH)
        GPIO.output(triggerPin, GPIO.LOW)


def _setup_():
    global rServo
    global lServo
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RIGHT_SERVO_PIN, GPIO.OUT)
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

    GPIO.setup(CON_PIN, GPIO.OUT)
    GPIO.output(CON_PIN, GPIO.HIGH)
    time.sleep(.001)
    GPIO.setup(CON_PIN, GPIO.IN)

    _offset_calibrate_()
    return True


def _read_arduino_(pin):
    min_value = 106/324
    max_value = 106/106
    if pin == CON_PIN:
        return GPIO.input(CON_PIN)
    else:
        GPIO.setup(CON_PIN, GPIO.OUT)
        GPIO.output(CON_PIN, GPIO.HIGH)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
        count = 0
        time.sleep(.1)

        GPIO.setup(pin, GPIO.IN)
        while GPIO.input(pin) == GPIO.LOW:
            count += 1
        count = 106/count
        GPIO.output(CON_PIN, GPIO.LOW)
        GPIO.setup(CON_PIN, GPIO.IN)
        if pin == YAW:
            return (count - min_value) * (180 - (-180) / ((max_value - min_value) + -180))
        else:
            return (count - min_value) * (90 - 0) / ((max_value - min_value) + 0)
# startup
_setup_()
lServo = GPIO.PMW(LEFT_SERVO_PIN, 100)
rServo = GPIO.PMW(RIGHT_SERVO_PIN, 100)
# main loop
while True:
    leftServo, rightServo = _update_cameras_()
    getPitch = _read_arduino_(PITCH)
    getYaw = _read_arduino_(YAW)
    _set_servos_(rightServo, leftServo)
    distance, angle = _distance_and_yaw_cal_(rightServo, leftServo)
    targetPitch = _calc_angle_(distance)
    canFire = _motor_control_(getYaw, angle, getPitch, targetPitch)
    _user_inter_face_(canFire)
