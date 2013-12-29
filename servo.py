# -*- coding: utf-8 -*-
from __future__ import division
from Adafruit_PWM_Servo_Driver import PWM

class Servo(object):
        def __init__(self, i2cAddress, channel, reversed=False, minAngle=5, maxAngle=190, callback=None):
                self._pwm=PWM(i2cAddress, debug=True) ##Set debug to False before release
                self.channel = channel
                self._pwm.setPWMFreq(50)
                
                self.pwm_min=90
                self.pwm_max=500
                self._actualpwm=None
                
                self.angle = None
                self.offset = 0
                self.reversed=reversed
                self.callback=callback
                
                if not (self.setMinAngle(minAngle) and self.setMaxAngle(maxAngle)):
                    sys.exit('Error initializing servo on channel #'+channel+'. Unable to set min/max values')

        def _isPWMValid(self, value):
                if value>=self.pwm_min and value<=self.pwm_max:
                        return True
                else:
                        return False

        def _isAngleWithinLimits(self, angle):
                ##Servo's angle limit can be less than a valid pwm value
                if angle>=self.minangle and angle<=self.maxangle:
                        return True
                else:
                        return False

        def checkServoAngle(self, angle):
                ##Assumes not for start
                result=False
                pwm=self._convAngleToPWM(angle)
                if _isAngleWithinLimits(angle) and _isPWMValid(pwm):
                        result=True
                return result

        def _setPWM(self, value):
                self._pwm.setPWM(self.channel, 0, value)
                self._actualpwm=value

        def _convAngleToPWM(self, pwm):
                return int(0.0022*pwm**2-2.416*pwm+500.42)
        
        def setMinAngle(self, angle):
                if self.checkServoAngle(angle):
                        self.minangle=angle
                        return True
                else:
                        print('Impossible to set Min angle of channel #'+self.channel+' to '+angle+'º!')
                        return False
                
        def setMaxAngle(self, angle):
                if self.checkServoAngle(angle):
                        self.maxangle=angle
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

        def setOffset(self, angleOffset):
                #Sets offset angle. Returns True if offset successful
                oldOffset=self.offset
                self.offset=angleOffset
                #Trying to update servo position according new offset angle
                if self.setAngle(self.angle):
                        return True
                else:
                        self.offset=oldOffset
                        return False

        def getOffset(self):
                return self.offset

        def setAngle(self, angle):
                ##Executes order to move servo to specified angle
                ##Returns False if not possible or True if OK
                offsetAngle=self.offset+angle
                if not self.checkServoAngle(offsetAngle):
                        #Angle not within servo range
                        return False
                else:
                        pwmvalue=self._convAngleToPWM(offsetAngle)
                        self._setPWM(pwmvalue)
                        self.angle=value
                        if self.callback is not None: self.callback
                        return True

        def incAngle(self, angleInc):
                return self.setServoAngle(self.calcAngleFromInc(angleInc))
        
        def getAngle(self):
                return self.angle

        def refresh(self):
                return self.setAngle(self.angle)
