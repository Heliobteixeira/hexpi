# -*- coding: utf-8 -*-
from __future__ import division
import sys
from Adafruit_PWM_Servo_Driver import PWM

class Servo(object):
        def __init__(self, i2cAddress, channel, pwm_min, pwm_max, reversed=False, minAngle=-90, maxAngle=90, callback=None):
                self._pwm=PWM(i2cAddress, debug=True) ##Set debug to False before release
                self.channel = channel
                self._pwm.setPWMFreq(60)

                
                self.pwm_min=pwm_min #pwm_min: HS-485HB->158 HS-645MG->149
                self.pwm_max=pwm_max #pwm_max: HS-485HB->643 HS-645MG->651
                self._actualpwm=None
                
                self.angle = None
                self.angleThrow = 180 #Maximum angle throw of servo end-to-end 
                self.offset = 0
                self.reversed=reversed
                self.callback=callback

                if (maxAngle-minAngle>self.angleThrow):
                        sys.exit('Result of maxAngle-minAngle exceeds predefined servo throw of %s' % self.angleThrow)                                            
                elif not (self.setMinAngle(minAngle) and self.setMaxAngle(maxAngle)):
                        sys.exit('Error initializing servo on channel #%s. Unable to set min/max values' % channel)

        def _isPWMValid(self, value):
                if value>=self.pwm_min and value<=self.pwm_max:
                        return True
                else:
                        print('PWM=%s is not valid. It is not between min/max pwm values of %s/%s' % (value, self.pwm_min, self.pwm_max))
                        return False

        def _isAngleWithinLimits(self, angle):
                ##Servo's angle limit can be less than a valid pwm value
                ##This function limits servo angle amplitude

                ##If min/max angles are not set they assume easy to superate values
                if not hasattr(self, 'minangle') or self.minangle is None:
                        self.minangle=-999999
                if not hasattr(self, 'maxangle') or self.maxangle is None:
                        self.maxangle=999999
                        
                if angle>=self.minangle and angle<=self.maxangle:
                        return True
                else:
                        if angle<self.minangle: print('Angle:%sº is lower than defined min. angle:%s (channel#%s)' % (angle, self.minangle, self.channel))
                        if angle>self.maxangle: print('Angle:%sº is higher than defined max. angle:%s (channel#%s)' % (angle, self.maxangle, self.channel))
                        return False

        def checkServoAngle(self, angle):
                ##Assumes not for start
                result=False
                pwm=self._convAngleToPWM(angle)
                if self._isAngleWithinLimits(angle) and self._isPWMValid(pwm):
                        result=True
                return result

        def _setPWM(self, value):
                self._pwm.setPWM(self.channel, 0, value)
                self._actualpwm=value
                #print('PWM set to: %s' % value)

        def _convAngleToPWM(self, angle):
#                return int(0.0022*pwm**2-2.416*pwm+500.42)
                absAngle=angle+90
                return int(((absAngle*(self.pwm_max-self.pwm_min))/self.angleThrow)+self.pwm_min)
        
        def setMinAngle(self, angle):
                if self.checkServoAngle(angle):
                        self.minangle=angle
                        return True
                else:
                        print('Impossible to set Min angle of channel #%s to %sº!' % (self.channel, angle))
                        return False
                
        def setMaxAngle(self, angle):
                if self.checkServoAngle(angle):
                        self.maxangle=angle
                        return True
                else:
                        print('Impossible to set Max angle of channel #%s to %sº!' % (self.channel, angle))
                        return False

        def calcAngleFromInc(self, angleInc):
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

        def _calcOffsetAngle(self, angle):
                offsetAngle=self.offset+angle
                if self.reversed:
                        offsetAngle=-offsetAngle #Reverses angle in case of servo 'reversed'
                return offsetAngle
        
        def setAngle(self, angle):
                ##Executes order to move servo to specified angle
                ##Returns False if not possible or True if OK
                offsetAngle=self._calcOffsetAngle(angle)
                
                if not self.checkServoAngle(offsetAngle):
                        #Angle not within servo range
                        return False
                else:
                        pwmvalue=self._convAngleToPWM(offsetAngle)
                        self._setPWM(pwmvalue)
                        self.angle=angle #Sets the original non offseted angle
                        if self.callback is not None: self.callback()
                        return True

        def incAngle(self, angleInc):
                return self.setAngle(self.calcAngleFromInc(angleInc))
        
        def getAngle(self):
                return self.angle

        def refresh(self):
                return self.setAngle(self.angle)
