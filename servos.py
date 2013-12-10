from __future__ import division
from Adafruit_PWM_Servo_Driver import PWM

class Servos(object):
	def __init__(self, i2cAddress, femurAxisChannel, tibiaAxisChannel, pwmFreqHz):
		self.pwm = PWM(i2cAddress, debug=True)
		self.pwm.setPWMFreq(pwmFreqHz)
		self.femuraxis = femurAxisChannel
		self.tibiaaxis = tibiaAxisChannel

	def setFemurAxis(self, value):
		self.pwm.setPWM(self.femuraxis, 0, value)

	def setTibiaAxis(self, value):
		self.pwm.setPWM(self.tibiaaxis, 0, value)

	def convAngleToPWM(self, pwm):
                return 0.0022*pwm**2-2.416*pwm+500.42

        def setFemurToAngle(self, value):
                self.setFemurAxis(int(self.convAngleToPWM(value)))

        def setTibiaToAngle(self, value):
                self.setTibiaAxis(int(self.convAngleToPWM(value)))
