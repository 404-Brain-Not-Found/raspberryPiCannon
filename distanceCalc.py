import math
global distanceBetween = 10
def distanceCal(angle):
    return float((((distanceBetween / math.cos(angle))**2)+(distanceBetween)**2)**1/2)
