import json
import sys
##import numpy as np
import servo
import ik
import math

class HexLimb(object):
    ##Reformular com base nas alteracoes da classe Servo
    def __init__(self, I2C_ADDRESS, femurLength, tibiaLength, femurInv, tibiaInv):
        #Setting precision of Limb Positioning
        self.precision=1
        
        #Setting i2c address and servo channels
        self.femur = servo.Servo(I2C_ADDRESS, 1, femurInv)
        self.tibia = servo.Servo(I2C_ADDRESS, 2, tibiaInv)
        #Setting limb's lengths
        self.femur.length=femurLength
        self.tibia.length=tibiaLength
        
        #Setting min/max angles
        #
        #

        #Loading servo calibration values
        self.servoCalibration = None
        self.transform = None
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()

        #Positions Limb in Starting Position 
        self.setFemurAngle(45)
        self.setTibiaAngle(90)

    def setMinFemurAngle(self, angle):
        if self.femur.checkServoAngle(angle):
            self.femur.min_angle=angle
        if not hasattr(self.femur, 'min_angle'):
            sys.exit('femur minimum angle not defined!')
            
        
    def setMaxFemurAngle(self, angle):
        if self.femur.checkServoAngle(angle):
            self.femur.max_angle=angle
        if not hasattr(self.femur, 'max_angle'):
            sys.exit('femur maximum angle not defined!')

    def setFemurAngle(self, angle):
        if (self.femur.setServoAngle(angle)):
            self.femur.angle = angle
            self.calcPosition()

    def calcAbsFemurAngleFromInc(self, bendangle):
        bendangle=bendangle*self.femur.rev
        return self.femur.angle+bendangle

    def checkFemurBend(self, bendangle):
        absAngle=self.calcAbsFemurAngleFromInc(bendangle)
        if absAngle>self.femur.min_angle and absAngle<self.femur.max_angle:
            return True
        else:
            return False
            
    def bendFemur(self, angle):
        self.setFemurAngle(self.calcAbsFemurAngleFromInc(angle))

    def getFemurAngle(self):
        return self.femur.angle

    def setMinTibiaAngle(self, angle):
        if self.tibia.checkServoAngle(angle):
            self.tibia.min_angle=angle
        if not hasattr(self.tibia, 'min_angle'):
            sys.exit('tibia minimum angle not defined!')
        
    def setMaxTibiaAngle(self, angle):
        if self.tibia.checkServoAngle(angle):
            self.tibia.max_angle=angle
        if not hasattr(self.tibia, 'max_angle'):
            sys.exit('tibia maximum angle not defined!')
        
    def setTibiaAngle(self, angle):
        if (self.tibia.setServoAngle(angle)):
            self.tibia.angle = angle
            self.calcPosition()
            
    def calcAbsTibiaAngleFromInc(self, bendangle):
        bendangle=bendangle*self.tibia.rev
        return self.tibia.angle+bendangle

    def checkTibiaBend(self, bendangle):
        absAngle=self.calcAbsTibiaAngleFromInc(bendangle)
        if absAngle>self.tibia.min_angle and absAngle<self.tibia.max_angle:
            return True
        else:
            return False

    def bendTibia(self, angle):
        self.setTibiaAngle(self.calcAbsTibiaAngleFromInc(angle))

    def getTibiaAngle(self):
        return self.tibia.angle

    def calcPosition(self):
        L1=self.femur.length
        L2=self.tibia.length
        try:
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

    def distTo(self, x, y):
        return math.sqrt((x-self.x)**2+(y-self.y)**2)

    def moveTipTo(self, x, y):
        i=0
        while self.distTo(x,y)>self.precision and i<500:
                length1=self.femur.length
                length2=self.tibia.length
                alpha1=90-self.femur.angle
                alpha2=180-self.tibia.angle
                (deltaFemur, deltaTibia)=ik.ik2DOFJacobian(length1, length2, alpha1, alpha2, 0, 0, x, y)
                self.bendLimbJoints(deltaFemur, deltaTibia)
                i+=1
