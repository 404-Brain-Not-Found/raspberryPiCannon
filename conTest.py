import RPi.GPIO as GPIO
import time

pin = 5
GPIO.setup(pin, GPIO.OUT)
GPIO.output(pin, GPIO.LOW)
count = 0
time.sleep(.1)

GPIO.setup(pin, GPIO.IN)
while GPIO.input(pin) == GPIO.LOW:
    count += 1
print count
