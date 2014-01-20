import json
import numpy as np
import time
import hexlimb

class HexModel(object):
    def __init__(self):
        self.limbs={ 3: hexlimb.HexLimb(0x40, [0,1,2], [10,70,120], [1,1,1])
                    }
        
##        self.targetCalibration = None
##        self.servoCalibration = None
##        self.calibrationFile = 'calibration.json'
##        self._loadCalibration()

    def stepLimbIndex(self, limbIndex, wait=1):
        self.limbs[limbIndex].bendFemur(60)
        self.limbs[limbIndex].bendHip(60)
        time.sleep(wait)
        self.limbs[limbIndex].bendFemur(-60)
        time.sleep(wait)
        self.limbs[limbIndex].bendHip(-60)
        time.sleep(wait)
        
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

