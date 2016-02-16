import RPi.GPIO as GPIO
import time

#servo pin
servoPin = 1
#set up
def servoStart():
GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPin, GPIO.OUT)
    servo = GPIO.PMW(18, 100)
    servo.start(5)

def setServo(angle):
    duty = float(angle) / 10.0 +2 .5
    servo.ChangeDutyCycle(duty)
