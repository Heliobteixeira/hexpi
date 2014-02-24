import json
import numpy as np
import time
import hexlimb

class HexModel(object):
    def __init__(self):
        self.limbs={ 1: hexlimb.HexLimb(0x40, [0,1,2], [10,70,125], [0,0,1]),
		     2: hexlimb.HexLimb(0x40, [4,5,6], [10,70,125], [0,0,1]),
		     3: hexlimb.HexLimb(0x40, [8,9,10], [10,70,125], [0,0,1]),
		     4: hexlimb.HexLimb(0x60, [0,1,2], [10,70,125], [1,1,0]),
		     5: hexlimb.HexLimb(0x60, [4,5,6], [10,70,125], [1,1,0]),
		     6: hexlimb.HexLimb(0x60, [8,9,10], [10,70,125], [1,1,0]),
                    }
        
##        self.targetCalibration = None
##        self.servoCalibration = None
##        self.calibrationFile = 'calibration.json'
##        self._loadCalibration()
    def stepLimbsSequence(self, indexSequenceArray, wait=1):
        for limbIndex in indexSequenceArray:
            self.limbs[limbIndex].doStep(wait)

    def stretchUp(self, wait=1):
        for limbIndex, limb in self.limbs.items():
            limb.stretchUp()

        time.sleep(wait)
        self.standPosition()

    def stretchDown(self, wait=1):
        for limbIndex, limb in self.limbs.items():
            limb.stretchDown()

        time.sleep(wait)
        self.standPosition()

    def stretchFront(self, wait=1):
        for limbIndex, limb in self.limbs.items():
            limb.stretchFront()

        time.sleep(wait)
        self.standPosition()        

    def stretchBack(self, wait=1):
        for limbIndex, limb in self.limbs.items():
            limb.stretchBack()

        time.sleep(wait)
        self.standPosition()
        
    def contract(self, wait=1):
        for limbIndex, limb in self.limbs.items():
            limb.contract()

        time.sleep(wait)         

    def stretchAll(self, wait=1):
        self.contract()
        self.stretchFront()
        self.stretchBack()
        self.stretchDown()
        self.stretchUp()

    def standPosition(self):
        for limbIndex, limb in self.limbs.items():
            limb.defaultPosition()

    def stepLimbIndex(self, limbIndex, wait=1):
        self.limbs[limbIndex].bendFemur(60)
        self.limbs[limbIndex].bendHip(60)
        time.sleep(wait)
        self.limbs[limbIndex].bendFemur(-60)
        time.sleep(wait)
        self.limbs[limbIndex].bendHip(-60)
        time.sleep(wait)

    def rotateBody(self, wait=1):
        self.limbs[1].stepRotatedBack(80,30, False)
        self.limbs[6].stepRotatedFront(80,30, False)
        time.sleep(wait)
        self.limbs[3].stepRotatedBack(80,30, False)
        self.limbs[4].stepRotatedFront(80,30, False)
        self.limbs[1].stepRotatedFront(80,30, True)
        self.limbs[6].stepRotatedBack(80,30, True)
        time.sleep(wait)
        self.limbs[3].stepRotatedFront(80,30, True)
        self.limbs[4].stepRotatedBack(80,30, True)
        self.limbs[1].stepRotatedBack(80,30, False)
        self.limbs[6].stepRotatedFront(80,30, False)
        
    def setCalibration(self, targetCalibration, servoCalibration):
##        self.targetCalibration = targetCalibration
        self.servoCalibration = servoCalibration
##        self._generateTransform()
        self._saveCalibration()

    def getCalibration(self):
##        return self.targetCalibration, self.servoCalibration
        return self.servoCalibration        

    def _loadCalibration(self):
        ##TODO
        try:
            with open(self.calibrationFile, 'r') as file:
                cal = json.loads(file.read())
##                self.targetCalibration = cal['targetCalibration']
                self.servoCalibration = cal['servoCalibration']
        except IOError:
            pass

    def _saveCalibration(self):
        ##TODO
        with open(self.calibrationFile, 'w') as file:
##            file.write(json.dumps({'targetCalibration': self.targetCalibration, 'servoCalibration': self.servoCalibration }))
            file.write(json.dumps({'servoCalibration': self.servoCalibration }))

