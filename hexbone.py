import json
##import numpy as np
from servo import Servo
import math
import sys

class HexBone(Servo):
    ## Servo.__init__ override:
    def __init__(self, I2C_ADDRESS, channel, length, startangle, pwm_min, pwm_max, reversed=False, minangle=None, maxangle=None, offset=0, callback=None):
        super(HexBone, self).__init__(I2C_ADDRESS, channel, pwm_min, pwm_max, reversed, minangle, maxangle, offset, callback)
        
        #Setting limb's lengths
        self.length=length
        #Positions Limb in Starting Position: NOOOT
        #if not self.setangle(startangle):
        #    sys.exit('Error initializing servo on channel #%s. Unable to set startAngle' % channel)
        

    def getLength(self):
        return self.length

