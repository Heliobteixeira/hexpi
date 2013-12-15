import json
##import numpy as np
import servo
import ik
import math

class HexLimb(object):
    def __init__(self, I2C_ADDRESS, femurLength, tibiaLength, femurInv, tibiaInv):
        #Setting i2c address and servo channels
        self.femur = servo.Servo(I2C_ADDRESS, 1)
        self.tibia = servo.Servo(I2C_ADDRESS, 2)
        #Setting limb's lengths
        self.femur.length=femurLength
        self.tibia.length=tibiaLength
        #Setting min/max angles
        setMinFemurAngle(0)
        setMaxFemurAngle(200)
        setMinTibiaAngle(0)
        setMaxTibiaAngle(200)
        #Setting reversed joints
        if femurInv:
            self.femur.rev=-1
        else:
            self.femur.rev=1

        if tibiaInv:
            self.tibia.rev=-1
        else:
            self.tibia.rev=1

        #Loading servo calibration values
        self.servoCalibration = None
        self.transform = None
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()

        #Positions Limb in Starting Position 
        self.setFemurAngle(45)
        self.setTibiaAngle(130)

    def setMinFemurAngle(self, angle):
        if self.femur.checkServoAngle(angle):
            self.femur.min_angle=angle
        
    def setMaxFemurAngle(self, angle):
        if self.femur.checkServoAngle(angle):
            self.femur.max_angle=angle

    def setFemurAngle(self, angle):
        if (self.femur.setServoAngle(angle)):
            self.femur.angle = angle
            self.calcPosition()

    def calcAbsFemurAngleFromInc(self, bendangle):
        bendangle=bendangle*self.femur.rev
        return self.femur.angle+bendangle

    def checkFemurBend(self, bendangle):
        absAngle=calcAbsFemurAngleFromInc(bendangle)
        if absAngle>self.femur.min_angle & absAngle<self.femur.max_angle:
            return True
        else:
            return False
            
    def bendFemur(self, angle):
        self.setFemurAngle(calcAbsFemurAngleFromInc(angle))

    def getFemurAngle(self):
        return self.femur.angle

    def setMinTibiaAngle(self, angle):
        if self.tibia.checkServoAngle(angle):
            self.tibia.min_angle=angle
        
    def setMaxTibiaAngle(self, angle):
        if self.tibia.checkServoAngle(angle):
            self.tibia.max_angle=angle
        
    def setTibiaAngle(self, angle):
        if (self.tibia.setServoAngle(angle)):
            self.tibia.angle = angle
            self.calcPosition()
            
    def calcAbsTibiaAngleFromInc(self, bendangle):
        bendangle=bendangle*self.tibia.rev
        return self.tibia.angle+bendangle

    def checkTibiaBend(self, bendangle):
        absAngle=calcAbsTibiaAngleFromInc(bendangle)
        if absAngle>self.tibia.min_angle & absAngle<self.tibia.max_angle:
            return True
        else:
            return False

    def bendTibia(self, angle):
        self.setTibiaAngle(calcAbsTibiaAngleFromInc(angle))

    def getTibiaAngle(self):
        return self.tibia.angle

    def calcPosition(self):
        L1=self.femur.length
        L2=self.tibia.length
        a1=self.femur.angle
        a2=self.tibia.angle
        self.x=L1*math.cos(math.radians(a1))+L2*math.cos(math.radians(a1-a2))
        self.y=L1*math.sin(math.radians(a1))+L2*math.sin(math.radians(a1-a2))

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
        if checkFemurBend(femurBendAngle) & checkTibiaBend(tibiaBendAngle):
            self.bendFemur(femurBendAngle)
            self.bendTibia(tibiaBendAngle)

    def moveTipTo(self, x, y):
        i=0
        while i<300:
                length1=self.femur.length
                length2=self.tibia.length
                alpha1=90-self.femur.angle
                alpha2=180-self.tibia.angle
                (deltaFemur, deltaTibia)=ik.ik2DOFJacobian(length1, length2, alpha1, alpha2, 0, 0, x, y)
                #print("FemurInc ", deltaFemur)
                #print("TibiaInc", deltaTibia)
                self.incFemurAngle(deltaFemur)
                self.incTibiaAngle(deltaTibia)
                i+=1
            
