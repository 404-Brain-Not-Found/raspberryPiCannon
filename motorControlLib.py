import time
from rrb2 import *

rr = RRB2()


#univeral varibale
global sampleTime = 500
global maxOutput = 1
global minOuput = -1
global reverse = 0
global forward = 1
def milis():
    return time.time()*1000

#x axis control
global Xkp = 1
global Xki = 1
global Xkd = 1
global xITerm = 0
global xLastInput = 0
global xSetPoint = 240
global xLastRun = 0
global xDirection = 1
global xNoCorrection = False
def xIValue():
    Xki = Xki * .5
def XDValue():
    Xkd = Xkd / .5
def xControl(error):
    now = millis()-xlastRun
    if now >= sampleTime:

        #if ki or kd have change recompute them
        if Xki != Xki: xIValue()
        if Xkd != Xkd: xDValue()
        
        #compute all the working error variables
        xITerm = float(Xki * error)
        if error > maxOutput: xIterm = maxOutput
        elif error < minOutput: xITerm = minOutput
        dInput = Input - xLastInput

        #compute PID output
        output = Xkp * error + xITerm - Xkd * dInput
        if output > maxOutput: output = maxOutput
        elif output < minOutput: output = minOuput

        #chech if no movement is need
        if output == 0: xNoCorrection = True
        else xNoCorrection = False
        
        #remember some variables for next time
        xlastInput = Input
        xlastRun = now

        #set motor direction
        if output > 0: xDirection = reverse
        else xDirection = forward
        return math.abs(output)


#y axis control
global ykp = 1
global yki = 1
global ykd = 1
global yITerm = 0
global yLastInput = 0
global yLastRun = 0
global yNoCorrection = False
def yIValue():
    yki = yki * .5
def yDValue():
    ykd = ykd / .5
def yPID(Input, setPoint):
    now = millis()-ylastRun
    if now >= sampleTime:

        #if ki or kd have change recompute them
        if yki != yki: yIValue()
        if ykd != ykd: yDValue()
        
        #compute all the working error variables
        error = float(SetPoint - Input)
        yITerm = float(yki * error)
        if error > mayOutput: yIterm = mayOutput
        elif error < minOutput: yITerm = minOutput
        dInput = Input - yLastInput

        #compute PID output
        output = ykp * error + yITerm - ykd * dInput
        if output > mayOutput: output = maxOutput
        elif output < minOutput: output = minOuput

        #chech if no movement is need
        if output == 0: yNoCorrection = True
        else yNoCorrection = False
        
        #remember some variables for neyt time
        ylastInput = Input
        ylastRun = now

        #set motor direction
        if output > 0: yDirection = reverse
        else yDirection = forward
        return math.abs(output)
#control motors
def motorControl(xInput,yInput,ySetpoint):
    rr.set_motors(xPID(xInput),xDirection,yPID(yInput,ySetpoint),yDirection)
    if xNoCorrection and yNoCorrection: return True
    else return False
        
