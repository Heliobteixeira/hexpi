import json
##import numpy as np
import servo

class HexLimb(object):
    def __init__(self, I2C_ADDRESS, femurLength, tibiaLength, femurInv, tibiaInv):
        self.femur.servo = servo.Servo(I2C_ADDRESS, 1)
        self.tibia.servo = servo.Servo(I2C_ADDRESS, 2)
        self.femur.length=femurLength
        self.tibia.length=tibiaLength
        if femurInv:
            self.femur.rev=-1
        else:
            self.femur.rev=1

        if tibiaInv:
            self.tibia.rev=-1
        else:
            self.tibia.rev=1

        #Positions Limb in Starting Position 
        self.setFemurAngle(180)
        self.setTibiaAngle(0)
       
        self.servoCalibration = None
        self.transform = None
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()

    def setFemurAngle(self, value):
        if (self.femur.servo.setServoAngle(value)):
            self.femur.angle = self.femur.servo.angle

    def incFemurAngle(self, value):
        value=value*self.femur.rev
        self.setFemurAngle(self.femur.angle+value)

    def getFemurAngle(self):
        return self.femur.angle
        
    def setTibiaAngle(self, value):
        if (self.tibia.servo.setServoAngle(value)):
            self.tibia.angle = self.tibia.servo.angle

    def incTibiaAngle(self, value):
        value=value*self.tibia.rev
        self.setTibiaAngle(self.tibia.angle+value)

    def getTibiaAngle(self):
        return self.tibia.angle

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

