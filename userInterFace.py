import RPi.GPIO as GPIO

notReadyLedPin = 1
readyLedPin = 2
fireBottonPin = 3
triggerPin = 4
def userInterfaceSetup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(notReadyLedPin, GPIO.OUT)
    GPIO.setup(readyLedPin, GPIO.OUT)
    GPIO.setup(triggerPin, GPIO.OUT)
    break

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
    break

