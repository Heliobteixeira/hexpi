import json
##import numpy as np
from servo import Servo
import math

class HexBone(Servo):
    ##Aplicar conceito de inheritance...ver como se costuma fazer o init
    def __init__(self, I2C_ADDRESS, channel, length, minAngle, maxAngle, startAngle, reversed=False):
        
        #Setting i2c address and servo channels
        self = servo.Servo(I2C_ADDRESS, 1, minAngle, maxAngle, reversed)
        #Setting limb's lengths
        self.length=length

        #Positions Limb in Starting Position 
        self.setAngle(startAngle)

    def getAngle(self):
        return self.angle

    def refresh(self):
        self.setAngle(self.angle)

