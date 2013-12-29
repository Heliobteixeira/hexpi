import json
##import numpy as np
from servo import Servo
import math

class HexBone(Servo):
    ## Servo.__init__ override:
    def __init__(self, I2C_ADDRESS, channel, length, startAngle, reversed=False, minAngle=None, maxAngle=None, callback=None):       
        self = Servo(I2C_ADDRESS, channel, reversed, minAngle, maxAngle, callback)
        
        #Setting limb's lengths
        self.length=length
        #Positions Limb in Starting Position 
        self.setAngle(startAngle)

    def getLength(self):
        return self.length

