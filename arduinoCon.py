def writeNumber(value):
    bus.write_byte(ARDUINOADDRESS, value)
    return -1

def readNumber():
    return bus.read_byte(ARDUINOADDRESS)
def getGyroValue():
    writeNumber(True)
    return readNumber()
