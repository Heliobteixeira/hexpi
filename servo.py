# -*- coding: utf-8 -*-
from __future__ import division
from Adafruit_PWM_Servo_Driver import PWM

class Servo(object):
        def __init__(self, i2cAddress, channel, reversed=False):
                self._pwm=PWM(i2cAddress, debug=True)
                self._pwm.setPWMFreq(50)
                self.pwm_min=90
                self.pwm_max=500
                self._actualpwm=None
                self.channel = channel
                self.angle = None
                self.reversed=reversed
                if not (self.setMinAngle(5) and self.setMaxAngle(190)):
                    sys.exit('Error initializing servo on channel #'+channel+'. Unable to set min/max values')

        def _isPWMValid(self, value):
                if value>=self.pwm_min and value<=self.pwm_max:
                        return True
                else:
                        return False

        def _isAngleWithinLimits(self, angle):
                ##Servo's angle limit can be less than a valid pwm value
                if angle>=self.min_angle and angle<=self.max_angle
                        return True
                else:
                        return False

        def checkServoAngle(self, angle):
                ##Assumes not for start
                result=False
                pwm=self._convAngleToPWM(angle)
                if _isAngleWithinLimits(angle) and _isPWMValid(pwm)
                        result=True
                return result

        def _setPWM(self, value):
                if self._isPWMValid(value):
                        self._pwm.setPWM(self.channel, 0, value)
                        self._actualpwm=value
                        return True
                else:
                        return False

        def _convAngleToPWM(self, pwm):
                return int(0.0022*pwm**2-2.416*pwm+500.42)
        
        def setMinAngle(self, angle):
                if self.checkServoAngle(angle):
                        self.min_angle=angle
                        return True
                else:
                        print('Impossible to set Min angle of channel #'+self.channel+' to '+angle+'º!')
                        return False
                
        def setMaxAngle(self, angle):
                if self.checkServoAngle(angle):
                        self.max_angle=angle
                        return True
                else:
                        print('Impossible to set Max angle of channel #'+self.channel+' to '+angle+'º!')
                        return False

        def calcAngleFromInc(self, angle):
               if self.reversed:
                        angleInc=angleInc*-1
                return self.angle+angleInc
                
        def checkIncAngle(self, angleInc):
                endAngle=self.calcAngleFromInc(angleInc)
                return self.checkServoAngle(endAngle)

        def setAngle(self, angle):
                ##Executes order to move servo to specified angle
                ##Returns False if not possible or True if OK
                if not self.checkServoAngle(angle):
                        #Angle not within servo range
                        return=False
                else:                      
                        self.angle=value
                        return=True

        def incAngle(self, angleInc):
                return self.setServoAngle(self.calcAngleFromInc(angleInc))  
