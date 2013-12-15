import json
import numpy as np

class HexModel(object):
    def __init__(self, servos, servoMin, servoMax, servoCenter):
        self.servos = servos
        self.servoMin = servoMin
        self.servoMax = servoMax
        self.setFemurAxis(servoCenter) #Coloca os servos na posicao central
        self.setTibiaAxis(servoCenter)
##        self.targetCalibration = None
        self.servoCalibration = None
        self.transform = None
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()
##        self._generateTransform()


    def setCalibration(self, targetCalibration, servoCalibration):
##        self.targetCalibration = targetCalibration
        self.servoCalibration = servoCalibration
##        self._generateTransform()
        self._saveCalibration()

    def getCalibration(self):
##        return self.targetCalibration, self.servoCalibration
        return self.servoCalibration        

    def _loadCalibration(self):
        """Load calibration data from disk."""
        try:
            with open(self.calibrationFile, 'r') as file:
                cal = json.loads(file.read())
##                self.targetCalibration = cal['targetCalibration']
                self.servoCalibration = cal['servoCalibration']
        except IOError:
            pass

    def _saveCalibration(self):
        """Save calibration data to disk."""
        with open(self.calibrationFile, 'w') as file:
##            file.write(json.dumps({'targetCalibration': self.targetCalibration, 'servoCalibration': self.servoCalibration }))
            file.write(json.dumps({'servoCalibration': self.servoCalibration }))

