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
