import unittest
from hexmodel import HexModel

class HexModelTests(unittest.TestCase):
	def setUp(self):
		self.servos = TestServos()
		self.model = HexModel(self.servos, 150, 650, 400)

	def test_setfemuraxis_getfemuraxis(self):
		self.model.setFemurAxis(200)
		self.assertEqual(self.model.getFemurAxis(), 200)
		self.assertEqual(self.servos.femuraxis, 200)

	def test_settibiaaxis_gettibiaaxis(self):
		self.model.setTibiaAxis(200)
		self.assertEqual(self.model.getTibiaAxis(), 200)
		self.assertEqual(self.servos.tibiaaxis, 200)

	def test_setfemuraxis_out_of_bounds_raises_valueerror(self):
		self.assertRaises(ValueError, self.model.setFemurAxis, 10)
		self.assertRaises(ValueError, self.model.setFemurAxis, 700)

	def test_settibiaAxis_out_of_bounds_raises_valueerror(self):
		self.assertRaises(ValueError, self.model.setTibiaAxis, 10)
		self.assertRaises(ValueError, self.model.setTibiaAxis, 700)

	def test_axis_defaults_to_400(self):
		self.assertEqual(self.model.getFemurAxis(), 400)
		self.assertEqual(self.model.getTibiaAxis(), 400)
		self.assertEqual(self.servos.femuraxis, 400)
		self.assertEqual(self.servos.tibiaaxis, 400)

	def test_setcalibration_getcalibration(self):
##		targetCal = [{'x': 150, 'y': 150}, {'x': 450, 'y': 150}, {'x': 400, 'y': 300}, {'x': 200, 'y': 300}]
		servoCal = [{'x': 10, 'y': 10}, {'x': 50, 'y': 10}, {'x': 50, 'y': 50}, {'x': 10, 'y': 50}]
		self.model.setCalibration(servoCal)
		sc = self.model.getCalibration()
##		self.assertEqual(tc, targetCal)
		self.assertEqual(sc, servoCal)

	def test_setcalibration_saves_calibration(self):
##		targetCal = [{'x': 150, 'y': 150}, {'x': 450, 'y': 150}, {'x': 400, 'y': 300}, {'x': 200, 'y': 300}]
		servoCal = [{'x': 10, 'y': 10}, {'x': 50, 'y': 10}, {'x': 50, 'y': 50}, {'x': 10, 'y': 50}]
		self.model.setCalibration(servoCal)
		self.model = HexModel(self.servos, 150, 650, 400)
		sc = self.model.getCalibration()
##		self.assertEqual(tc, targetCal)
		self.assertEqual(sc, servoCal)

	def test_target(self):
##		targetCal = [{'x': 190, 'y': 190}, {'x': 555, 'y': 190}, {'x': 480, 'y': 525}, {'x': 240, 'y': 525}]
		servoCal = [{'x': 440, 'y': 298}, {'x': 340, 'y': 298}, {'x': 340, 'y': 220}, {'x': 440, 'y': 220}]
		#targetCal = [{"y": 89, "x": 143}, {"y": 89, "x": 516}, {"y": 437, "x": 447}, {"y": 435, "x": 190}]
		#servoCal = [{"y": 445, "x": 430}, {"y": 352, "x": 428}, {"y": 367, "x": 348}, {"y": 425, "x": 345}]
		self.model.setCalibration(servoCal)
##		self.model.target(190, 190)
		#self.model.target(144, 90)
		self.assertEqual(self.servos.femuraxis, 440)
		self.assertEqual(self.servos.tibiaaxis, 298)


class TestServos(object):
	def __init__(self):
		self.femuraxis = 0
		self.tibiaaxis = 0

	def setFemurAxis(self, value):
		self.femuraxis = value

	def setTibiaAxis(self, value):
		self.tibiaaxis = value


if __name__ == '__main__':
	unittest.main()
