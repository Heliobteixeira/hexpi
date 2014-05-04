import json
##import numpy as np
from servo import Servo
import math

class HexBone(Servo):
    ## Servo.__init__ override:
    def __init__(self, I2C_ADDRESS, channel, length, startAngle, pwm_min, pwm_max, reversed=False, minAngle=None, maxAngle=None, callback=None):       
        super(HexBone, self).__init__(I2C_ADDRESS, channel, pwm_min, pwm_max, reversed, minAngle, maxAngle, callback)
        
        #Setting limb's lengths
        self.length=length
        #Positions Limb in Starting Position
        if not self.setAngle(startAngle):
            sys.exit('Error initializing servo on channel #%s. Unable to set startAngle' % channel)
        

    def getLength(self):
        return self.length

