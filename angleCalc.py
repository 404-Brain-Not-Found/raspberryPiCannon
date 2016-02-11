import math
surfaceAreaOfBall = .8620530241
massOfBall = 0.0594
gravity = 9.8
def calcAngle(distance):
    acceleration = 
    insideSin = (gravity * distance)/ acceleration**2
    if math.fabs(insideSin) > 1:
        return "Out of Range"
    return math.degrees(math.sin(insideSin))/2

print calcAngle(100)
    
    
