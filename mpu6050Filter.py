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
#filter varibles
global bias = 0 #bias on pitch
global biasCalcStartTimer = 0
global
def calBias()
    
