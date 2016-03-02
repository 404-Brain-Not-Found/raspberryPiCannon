import RPi.GPIO as GPIO

PITCH_PIN = 4
DIGITAL_MAX = 255
DIGITAL_MIN = 0

def arduinoSetup():
    configureDitialPin(GPIO,PITCH_PIN,'input')

def digitalConvert(x, in_min, in_max, out_min, out_max):
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
def getGyroVaule():
    value = readDigitalPin(GPIO, PITCH_PIN)
    return digitalConvert(value, DIGITAL_MIN, DIGITAL_MAX, -90, 90)
