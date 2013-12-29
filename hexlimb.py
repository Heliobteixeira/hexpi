import json
import sys
##import numpy as np
from hexbone import HexBone
import ik
import math

class HexLimb(object):
    ##Reformular com base nas alteracoes da classe Servo
    def __init__(self, I2C_ADDRESS, femurLength, tibiaLength, femurInv, tibiaInv):
        #Setting precision of Limb Positioning
        self.precision=1
        
        #Setting i2c address and servo channels
        self.femur = HexBone(I2C_ADDRESS, 1, femurLength, 45, femurInv, 5, 190, self.calcPosition)
        self.tibia = HexBone(I2C_ADDRESS, 2, tibiaLength, 90, tibiaInv, 5, 190, self.calcPosition)
        
        #Loading servo calibration values
        self.servoCalibration = None
        self.transform = None
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()

        #Reset servo positions after servo calibration
        self.updatePositions()
        self.calcPosition()

    def setFemurAngle(self, angle):
        return self.femur.setAngle(angle)

    def checkFemurBend(self, bendangle):
        return self.femur.checkIncAngle(bendangle)
            
    def bendFemur(self, angle):
        return self.femur.incAngle(angle)

    def getAbsFemurAngle(self):
        return self.femur.angle

    def getNorFemurAngle(self):
        return 90-self.femur.angle
        
    def setTibiaAngle(self, angle):
        return self.tibia.setAngle(angle)

    def checkTibiaBend(self, bendangle):
        return self.tibia.checkIncAngle(bendangle)

    def bendTibia(self, angle):
        return self.tibia.incAngle(angle)

    def getAbsTibiaAngle(self):
        return self.tibia.angle

    def getNorTibiaAngle(self):
        return 180-self.tibia.angle

    def calcPosition(self):
        try:
            L1=self.femur.length
            L2=self.tibia.length
            a1=90-self.femur.angle#########!!!!!!
            a2=180-self.tibia.angle
            self.x=L1*math.cos(math.radians(a1))+L2*math.cos(math.radians(a1-a2))
            self.y=L1*math.sin(math.radians(a1))+L2*math.sin(math.radians(a1-a2))
        except:
            return False
        else:
            return True

    def setCalibration(self, targetCalibration, servoCalibration):
        self.servoCalibration = servoCalibration
        self._saveCalibration()

    def getCalibration(self):
        return self.servoCalibration        

    def _loadCalibration(self):
        """Load calibration data from disk."""
        try:
            with open(self.calibrationFile, 'r') as file:
                cal = json.loads(file.read())
                self.servoCalibration = cal['servoCalibration']
        except IOError:
            pass

    def _saveCalibration(self):
        """Save calibration data to disk."""
        with open(self.calibrationFile, 'w') as file:
            file.write(json.dumps({'servoCalibration': self.servoCalibration }))

    def bendLimbJoints(self, femurBendAngle, tibiaBendAngle):
        if self.checkFemurBend(femurBendAngle) and self.checkTibiaBend(tibiaBendAngle):
            self.bendFemur(femurBendAngle)
            self.bendTibia(tibiaBendAngle)
            return True
        else:
            return False

    def distTo(self, x, y):
        return math.sqrt((x-self.x)**2+(y-self.y)**2)

    def moveTipTo(self, x, y):
        i=0
        lastMoveFine=True
        while self.distTo(x,y)>self.precision and i<500 and lastMoveFine:
                length1=self.femur.length
                length2=self.tibia.length
                alpha1=self.getNorFemurAngle()
                alpha2=self.getNorTibiaAngle()
                (deltaFemur, deltaTibia)=ik.ik2DOFJacobian(length1, length2, alpha1, alpha2, 0, 0, x, y)
                lastMoveFine=self.bendLimbJoints(deltaFemur, deltaTibia)
                i+=1

    def updatePositions(self):
        self.setFemurAngle(self.femur.angle)
        self.setTibiaAngle(self.tibia.angle)
