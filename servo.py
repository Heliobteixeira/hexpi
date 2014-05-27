# -*- coding: utf-8 -*-
from __future__ import division
import sys
from Adafruit_PWM_Servo_Driver import PWM

class Servo(object):
        def __init__(self, i2cAddress, channel, pwm_min, pwm_max, reversed=False, offset=0, minangle=-90, maxangle=90, callback=None):
                self.i2cAddress=i2cAddress
                self._pwm=PWM(self.i2cAddress, debug=False) ##Set debug to False before release

                self.channel = channel
                self._pwm.setPWMFreq(60)

                
                self.pwm_min=pwm_min #pwm_min: HS-485HB->158 HS-645MG->149
                self.pwm_max=pwm_max #pwm_max: HS-485HB->643 HS-645MG->651
                self._actualpwm=None
                
                self.angle = None
                self.anglethrow = 180 #Maximum angle throw of servo end-to-end
                self.offset = offset
                self.reversed=reversed
                self.callback=callback

                if (maxangle-minangle>self.anglethrow):
                        sys.exit('Result of maxAngle-minAngle exceeds predefined servo throw of %s' % self.anglethrow)
                elif not (self.setminangle(minangle) and self.setmaxangle(maxangle)):
                        sys.exit('Error initializing servo on channel #%s. Unable to set min/max values' % channel)

        def _ispwmvalid(self, value):
                if self.pwm_min <= value <= self.pwm_max:
                        return True
                else:
                        print('PWM=%s is not valid. It is not between min/max pwm values of %s/%s' % (value, self.pwm_min, self.pwm_max))
                        return False

        def _isanglewithinlimits(self, angle):
                ##Servo's angle limit can be less than a valid pwm value
                ##This function limits servo angle amplitude

                ##If min/max angles are not set they assume easy to superate values
                if not hasattr(self, 'minangle') or self.minangle is None:
                        self.minangle=-999999
                if not hasattr(self, 'maxangle') or self.maxangle is None:
                        self.maxangle=999999
                        
                if self.minangle <= angle <= self.maxangle:
                        return True
                else:
                        if angle<self.minangle: print('Angle:%sº is lower than defined min. angle:%s (channel#%s)' % (angle, self.minangle, self.channel))
                        if angle>self.maxangle: print('Angle:%sº is higher than defined max. angle:%s (channel#%s)' % (angle, self.maxangle, self.channel))
                        return False

        def checkservoangle(self, angle):
                ##Assumes not for start
                result=False
                pwm=self._convangletopwm(angle)
                if self._isanglewithinlimits(angle) and self._ispwmvalid(pwm):
                        result=True
                return result

        def _setpwm(self, value):
                self._pwm.setPWM(self.channel, 0, value)
                self._actualpwm=value
                #print('PWM set to: %s' % value)

        def _convangletopwm(self, angle):
#                return int(0.0022*pwm**2-2.416*pwm+500.42)
                absAngle=angle+90
                return int(((absAngle*(self.pwm_max-self.pwm_min))/self.anglethrow)+self.pwm_min)
        
        def setminangle(self, angle):
                if self.checkservoangle(angle):
                        self.minangle=angle
                        return True
                else:
                        print('Impossible to set Min angle of channel #%s to %sº!' % (self.channel, angle))
                        return False
                
        def setmaxangle(self, angle):
                if self.checkservoangle(angle):
                        self.maxangle=angle
                        return True
                else:
                        print('Impossible to set Max angle of channel #%s to %sº!' % (self.channel, angle))
                        return False

        def calcanglefrominc(self, angleinc):
                return self.angle+angleinc
                
        def checkincangle(self, angleinc):
                endangle=self.calcanglefrominc(angleinc)
                return self.checkservoangle(endangle)

        def setoffset(self, angleoffset):
                #Sets offset angle. Returns True if offset successful
                oldoffset=self.offset
                
                self.offset=angleoffset

                #Trying to update servo position according new offset angle
                if self.setangle(self.angle):
                        print('Channel %s (%s)> offset: %s' % (self.channel, self.i2cAddress, self.offset))
                        return True
                else:
                        print('Error applying offsets')
                        self.offset=oldoffset
                        return False

        def incoffset(self, angleoffset):
                #Sets offset angle. Returns True if offset successful
                oldoffset=self.offset
                
                self.offset=self.offset+angleoffset

                #Trying to update servo position according new offset angle
                if self.setangle(self.angle):
                        print('Channel %s (%s)> offset: %s' % (self.channel, self.i2cAddress, self.offset))
                        return True
                else:
                        print('Error applying offsets')
                        self.offset=oldoffset
                        return False

        @property
        def getOffset(self):
                return self.offset

        def _calcoffsetangle(self, angle):
                offsetangle=self.offset+angle
                if self.reversed:
                        offsetangle=-offsetangle #Reverses angle in case of servo 'reversed'
                return offsetangle
        
        def setangle(self, angle):
                ##Executes order to move servo to specified angle
                ##Returns False if not possible or True if OK
                offsetAngle=self._calcoffsetangle(angle)
                
                if not self.checkservoangle(offsetAngle):
                        #Angle not within servo range
                        return False
                else:
                        pwmvalue=self._convangletopwm(offsetAngle)
                        self._setpwm(pwmvalue)
                        self.angle=angle #Sets the original non offseted angle >>CONFIRM
                        if self.callback is not None: self.callback()
                        return True
                
        def setangletominormax(self, minmax):
                #Sets servo to its min/max (-1/1) angle
                if minmax==1: ##Max
                        print('Setting channel %s to Max' % self.channel)
                        if self.reversed:
                                angle=self.minangle
                        else:
                                angle=self.maxangle
                else: ##Min
                        print('Setting channel %s to Min' % self.channel)
                        if self.reversed:
                                angle=self.maxangle
                        else:
                                angle=self.minangle
                        
                if not self.checkservoangle(angle):
                        #Angle not within servo range
                        return False
                else:
                        pwmvalue=self._convangletopwm(angle)
                        self._setpwm(pwmvalue)
                        self.angle=angle 
                        if self.callback is not None: self.callback()
                        return True
                
        def incangle(self, angleinc):
                return self.setangle(self.calcanglefrominc(angleinc))
        
        @property
        def getangle(self):
                return self.angle

        def powerOff(self):
                self._setpwm(0)
                
        def powerOn(self):
                self.setangle(self.angle)

        def refresh(self):
                return self.setangle(self.angle)
