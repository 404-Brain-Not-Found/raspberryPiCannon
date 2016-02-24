import smbus
import time

bus = smbus.SMBus(1)

ARDUINOADDRESS = 0x04

def writeNumber(value):
    bus.write_byte(ARDUINOADDRESS, value)
    return -1

def readNumber():
    return bus.read_byte(ARDUINOADDRESS)
def getGyroVaule():
    writeNumber(True)
    return readNumber()
