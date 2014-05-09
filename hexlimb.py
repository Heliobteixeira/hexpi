import json
import sys
##import numpy as np
from hexbone import HexBone
import ik
import math
from time import sleep

class HexLimb(object):
                    
    ##Reformular com base nas alteracoes da classe Servo
    def __init__(self, I2C_ADDRESS, bonesChannelsArray, bonesLengthArray, revArray):
        #0->Hip;1->Femur;2->Tibia

        ## Hexbone's min/max default angles
        defaultMinAngle=-90
        defaultMaxAngle=+90
        
        #Setting precision of Limb Positioning
        self.precision=1
        
        #Setting i2c address and servo channels
        self.hip   = HexBone(I2C_ADDRESS, bonesChannelsArray[0], bonesLengthArray[0], 0, 158, 643, revArray[0], defaultMinAngle, defaultMaxAngle, self.calcPosition)
        self.femur = HexBone(I2C_ADDRESS, bonesChannelsArray[1], bonesLengthArray[1], 0, 149, 651, revArray[1], defaultMinAngle, defaultMaxAngle, self.calcPosition)
        self.tibia = HexBone(I2C_ADDRESS, bonesChannelsArray[2], bonesLengthArray[2], 0, 158, 643, revArray[2], defaultMinAngle, defaultMaxAngle, self.calcPosition)

        #Reset servo positions after servo calibration
        self.updatePositions()
        self.calcPosition()

    ##Code hip movement functions
    def setFemurAngle(self, angle):
        return self.femur.setAngle(angle)

    def checkFemurBend(self, bendangle):
        return self.femur.checkIncAngle(bendangle)
            
    def bendFemur(self, angle):
        return self.femur.incAngle(angle)

    def getAbsFemurAngle(self):
        return self.femur.angle

    def getNorFemurAngle(self):
        return self.femur.angle

        
    def setTibiaAngle(self, angle):
        return self.tibia.setAngle(angle)

    def checkTibiaBend(self, bendangle):
        return self.tibia.checkIncAngle(bendangle)

    def bendTibia(self, angle):
        return self.tibia.incAngle(angle)

    def getAbsTibiaAngle(self):
        return self.tibia.angle

    def getNorTibiaAngle(self):
        return 90+self.tibia.angle


    def setHipAngle(self, angle):
        return self.hip.setAngle(angle)

    def checkHipBend(self, bendangle):
        return self.hip.checkIncAngle(bendangle)
            
    def bendHip(self, angle):
        return self.hip.incAngle(angle)

    def getAbsHipAngle(self):
        return self.hip.angle

    def getNorHipAngle(self):
        return self.hip.angle

    def setOffsets(self, hipOffset=None, femurOffset=None, tibiaOffset=None):
        success=True
        if hipOffset is not None:
            if not self.hip.setOffset(hipOffset):
                success=False
                
        if femurOffset is not None:
            if not self.femur.setOffset(femurOffset):
                success=False
                
        if tibiaOffset is not None:
            if not self.tibia.setOffset(tibiaOffset):
                success=False 
        
        return success
    
    def getCurrentPosition(self):
        print(' Hip: %2.1f   | Femur: %2.1f    | Tibia: %2.1f    |  x=%.2f;y=%.2f' % (self.getNorHipAngle(),
                                                              self.getNorFemurAngle(),
                                                              self.getNorTibiaAngle(),
                                                              self.x, self.y)) 
        
    def calcPosition(self):
        #Callback function (when setAngle is called) to update tip position
        try:
            L1=self.femur.length
            L2=self.tibia.length
            a1=self.getNorFemurAngle()
            a2=self.getNorTibiaAngle()
            self.x=L1*math.cos(math.radians(a1))+L2*math.cos(math.radians(a1-a2))
            self.y=L1*math.sin(math.radians(a1))+L2*math.sin(math.radians(a1-a2))
        except:
            return False
        else:
            return True        

    def calcCurrentDistToPoint(self, x, y):
        xi=self.x
        yi=self.y
        return float( ((x-xi)**2+(y-yi)**2)**(1/2.0) )
    
    def bendLimbJoints(self, femurBendAngle, tibiaBendAngle):
        #Bends Femur and Tibia simultaneously
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
                print('Distance to target: %f.2' % self.calcCurrentDistToPoint(x,y))
                i+=1
        if lastMoveFine:
            print('******Success!****** \n-NbrIterations: %i\nCurrDistToTarget:%f.2' % (i, self.calcCurrentDistToPoint(x,y)))
        else:
            print('...Unable To Reach... \n-NbrIterations: %i\nCurrDistToTarget:%f.2' % (i, self.calcCurrentDistToPoint(x,y)))
        print('Current Position:')
        self.getCurrentPosition()
        

    def updatePositions(self):
        self.setFemurAngle(self.femur.angle)
        self.setTibiaAngle(self.tibia.angle)

## Testing Functions:
    def defaultPosition(self):
        self.setTibiaAngle(0)
        self.setFemurAngle(0)
        self.setHipAngle(0)

    def doStep(self, wait=1):
        ##Position all Limbs in known position because uses bend instead of setAngle
        ##Change to setAngle!!!
        femurAngle=30
        tibiaAngle=30
        hipAngle=20
        self.defaultPosition()
        self.bendFemur(femurAngle)
        self.bendTibia(-tibiaAngle)
        self.bendHip(hipAngle)
        sleep(wait)       
        self.bendFemur(-femurAngle)
        self.bendTibia(tibiaAngle)
        sleep(wait)
        self.bendHip(-hipAngle)
        sleep(wait)

    def stretchUp(self):
        self.setFemurAngle(40)
        self.setTibiaAngle(40)

    def stretchDown(self):
        self.setFemurAngle(-40)
        self.setTibiaAngle(50)

    def contract(self):
        self.setFemurAngle(40)
        self.setTibiaAngle(-35)        

    def stretchFront(self):
        self.setHipAngle(20)
        
    def stretchBack(self):
        self.setHipAngle(-20)

    def lift(self):
        self.setFemurAngle(30)

    def stepRotatedFront(self, hipAngle):
        ##self.defaultPosition()
        self.bendHip(hipAngle) ##TODO: Interpolate movement

    def stepRotatedBack(self, hipAngle):
        ##self.defaultPosition()
        self.bendHip(-hipAngle) ##TODO: Interpolate movement
