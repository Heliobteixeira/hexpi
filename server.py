import json, sys
<<<<<<< HEAD
=======

>>>>>>> 860beeef3ffd7b92a12d7b86086a64cdc4402772
import hexmodel

# Cat laser toy configuration
SERVO_I2C_ADDRESS 	= 0x40		# I2C address of the PCA9685-based servo controller
SERVO_XAXIS_CHANNEL = 1 		# Channel for the x axis rotation which controls laser up/down
SERVO_YAXIS_CHANNEL = 2			# Channel for the y axis rotation which controls laser left/right
SERVO_PWM_FREQ 		= 50 		# PWM frequency for the servos in HZ (should be 50)
SERVO_MIN 			= 90		# Minimum rotation value for the servo, should be -90 degrees of rotation.
SERVO_MAX 			= 500		# Maximum rotation value for the servo, should be 90 degrees of rotation.
<<<<<<< HEAD
SERVO_CENTER		= 300		# Center value for the servo, should be 0 degrees of rotation.

# Setup the real servo when running on a Raspberry Pi
import servos
servos = servos.Servos(SERVO_I2C_ADDRESS, SERVO_XAXIS_CHANNEL, SERVO_YAXIS_CHANNEL, SERVO_PWM_FREQ)

hexmodel = hexmodel.HexModel(servos, SERVO_MIN, SERVO_MAX, SERVO_CENTER)


=======
SERVO_CENTER		= 280		# Center value for the servo, should be 0 degrees of rotation.


import servos
servos = servos.Servos(SERVO_I2C_ADDRESS, SERVO_XAXIS_CHANNEL, SERVO_YAXIS_CHANNEL, SERVO_PWM_FREQ)

hexmodel = model.HexModel(servos, SERVO_MIN, SERVO_MAX, SERVO_CENTER)
>>>>>>> 860beeef3ffd7b92a12d7b86086a64cdc4402772
