# -*- coding: utf-8 -*-
from __future__ import division
from Adafruit_PWM_Servo_Driver import PWM

class Servo(object):
        def __init__(self, i2cAddress, channel):
                self._pwm=PWM(i2cAddress, debug=True)
                self._pwm.setPWMFreq(50)
                self.pwm_min=90
                self.pwm_max=500
                self._actualpwm=None
                self.channel = channel
                self.angle = None

        def _checkServoPWM(self, value):
                if value>=self.pwm_min and value<=self.pwm_max:
                        return True
                else:
                        return False

        def checkServoAngle(self, angle):
                pwmvalue=self._convAngleToPWM(angle)
                if self._checkServoPWM(pwmvalue):
                        return True
                else:
                        return False

        def _setServoPWM(self, value):
                if self._checkServoPWM(value):
                        self._pwm.setPWM(self.channel, 0, value)
                        self._actualpwm=value
                        return True
                else:
                        return False

        def _convAngleToPWM(self, pwm):
                return 0.0022*pwm**2-2.416*pwm+500.42

        def setServoAngle(self, value):
                if self._setServoPWM(int(self._convAngleToPWM(value)))==1:                        
                        self.angle=value
                        return True
                else:
                        print("Servo angle=", value, "º out of bounds on channel:", self.channel)
                        return False

        def incServoAngle(self, value):
                self.setServoAngle(self.angle+value)  
