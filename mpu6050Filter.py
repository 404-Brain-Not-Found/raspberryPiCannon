import time
import math
import mpu6050
#set up mpu
mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.dmpDMPEnabled(True)
#raw values
global rawYaw = 0
global rawPitch = 0
global rawRoll = 0
def readMPU():
# Get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus()
  
    if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently) 
        # get current FIFO count
        fifoCount = mpu.getFIFOCount()
        
        # check for overflow (this should never happen unless our code is too inefficient)
        if fifoCount == 1024:
            # reset so we can continue cleanly
            mpu.resetFIFO()
            print('FIFO overflow!')
            
            
        # wait for correct available data length, should be a VERY short wait
        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()
        
        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        ypr = mpu.dmpGetYawPitchRoll(q, g)
        
        rawYaw =(ypr['yaw'] * 180 / math.pi),
        rawPitch =(ypr['pitch'] * 180 / math.pi),
        rawRoll = (ypr['roll'] * 180 / math.pi)
    
        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)        
        fifoCount -= packetSize 
#filter code
#bias calc
global bias = 0 #bias on pitch
global biasCalcStartTimer = 0
global hasRun = False
global biasCalcReadings[20]
global biasCount = 0
global biasFirstRead = 0
def calBias():
    if not hasRun:
        biasCalcStartTimer = time.time()
        hasRun = True
        biasFristRead = filteredPitch()
    else:
        if time.time() - biasCalcStartTimer > 10:
            hasRun = False
            biasCount = 0
        else:
            if biasCount == 20:
                error = 0
                for i in range(10):
                    error =+ biasCalcReadings[i] - biasCalcReadings[i + 1]
                bias += error/20
                hasRun = False
            else:
                diasCalcReadings[biasCount] = filteredPitch()
                biasCount += 1
#filter
def filteredPitch():
    return readMPU() - bias
#main loop
def updateMPU(stopped)
    if stopped: calBias()
    else: return filteredPitch()

    
