import sys
##import numpy as np
from hexbone import HexBone
from InverseKinematics import ik
import math
from time import sleep

class HexLimb(object):
                    
    ##Reformular com base nas alteracoes da classe Servo
    def __init__(self, I2C_ADDRESS, bonesChannelsArray, bonesLengthArray, revArray, startPosition):
        #0->Hip;1->Femur;2->Tibia
        
        ## Hexbone's min/max default angles
        defaultMinAngle=-90
        defaultMaxAngle=+90
        
        #Setting i2c address and servo channels
        self.hip   = HexBone(I2C_ADDRESS, bonesChannelsArray[0], bonesLengthArray[0], 0, 158, 643, revArray[0], defaultMinAngle, defaultMaxAngle, self.calcPosition)
        self.femur = HexBone(I2C_ADDRESS, bonesChannelsArray[1], bonesLengthArray[1], 0, 149, 651, revArray[1], defaultMinAngle, defaultMaxAngle, self.calcPosition)
        self.tibia = HexBone(I2C_ADDRESS, bonesChannelsArray[2], bonesLengthArray[2], 0, 158, 643, revArray[2], defaultMinAngle, defaultMaxAngle, self.calcPosition)

        self.origin=[0,0,0] #Correct this along the code and include as parameter of __init__
        self.startPosition=startPosition
        #Reset servo positions after servo calibration
        self.defaultPosition()
        self.calcPosition()

    def powerOff(self):
        self.hip.powerOff()
        self.femur.powerOff()
        self.tibia.powerOff()

    def powerOn(self):
        self.hip.powerOn()
        self.femur.powerOn()
        self.tibia.powerOn()        

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
        return self.tibia.setAngle(angle-90)

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
    
    def printPosition(self):
        print(' Hip: %2.1f   | Femur: %2.1f    | Tibia: %2.1f    |  x=%.2f;y=%.2f;z=%.2f' % (self.getNorHipAngle(),
                                                              self.getNorFemurAngle(),
                                                              self.getNorTibiaAngle(),
                                                              self.x, self.y, self.z)) 
        
    def calcPosition(self):
        #Callback function (when setAngle is called) to update tip position
        try:
            L1=self.hip.length
            L2=self.femur.length
            L3=self.tibia.length
            alpha=math.radians(self.getNorHipAngle())
            beta=math.radians(self.getNorFemurAngle())  # RADIANS!!!
            gamma=math.radians(self.getNorTibiaAngle())
##            self.x=L1*math.cos(math.radians(a1))+L2*math.cos(math.radians(a1-a2))
##            self.z=L1*math.sin(math.radians(a1))+L2*math.sin(math.radians(a1-a2))
            
            projL=L1+L2*math.cos(beta)+L3*math.cos(beta-gamma)
            self.x=self.origin[0] + math.cos(alpha)*projL
            self.y=self.origin[1] + math.sin(alpha)*projL
            self.z=self.origin[2] + L2*math.sin(beta) + L3*math.sin(beta-gamma)
        except:
            return False
        else:
            return True        
    
    def bendLimbJoints(self, hipBendAngle, femurBendAngle, tibiaBendAngle):
        #Bends Femur and Tibia simultaneously
        if self.checkHipBend(hipBendAngle) and self.checkFemurBend(femurBendAngle) and self.checkTibiaBend(tibiaBendAngle):
            self.bendHip(hipBendAngle)
            self.bendFemur(femurBendAngle)
            self.bendTibia(tibiaBendAngle)
            return True
        else:
            return False

    def distTo(self, x, y, z):
        return math.sqrt((x-self.x)**2+(y-self.y)**2+(z-self.z)**2)

    def iterateHipFemurTibiaBend(self,targetPosition, maxAngleDisp):
        origin=[0,0,0]
        L1=self.hip.length
        L2=self.femur.length
        L3=self.tibia.length
        alpha=math.radians(self.getNorHipAngle())
        beta=math.radians(self.getNorFemurAngle())
        gamma=math.radians(self.getNorTibiaAngle())
        (deltaHip, deltaFemur, deltaTibia)=ik.ikJacobian3DOF(L1, L2, L3, alpha, beta, gamma, origin, targetPosition)
        (deltaHip, deltaFemur, deltaTibia)=(math.degrees(deltaHip), math.degrees(deltaFemur), math.degrees(deltaTibia))
        
        currMaxAngle=max(abs(deltaHip), abs(deltaFemur), abs(deltaTibia))
        
        if currMaxAngle>maxAngleDisp:
            fact=maxAngleDisp/currMaxAngle
            (deltaHip, deltaFemur, deltaTibia)=(deltaHip*fact, deltaFemur*fact, deltaTibia*fact)
            
        ##print('Bend Angles: Hip: %s, Femur:%s, Tibia:%s' % (deltaHip, deltaFemur, deltaTibia))
        return (deltaHip, deltaFemur, deltaTibia)

    def calculateHipFemurTibiaBend(self, targetPosition):
        origin=[0,0,0]
        L1=self.hip.length
        L2=self.femur.length
        L3=self.tibia.length
    
        ## Fetching current angles position:
        startAlpha=math.radians(self.getNorHipAngle())
        startBeta=math.radians(self.getNorFemurAngle())
        startGamma=math.radians(self.getNorTibiaAngle())

        ## Calculating final angles position:
        (endAlpha, endBeta, endGamma)=ik.ikAnalytical3DOF(L1, L2, L3, origin, targetPosition)

        deltaAlpha=math.degrees(endAlpha-startAlpha)
        deltaBeta=math.degrees(endBeta-startBeta)
        deltaGamma=math.degrees(endGamma-startGamma)

        #print('Will rotate:'+str((deltaAlpha, deltaBeta, deltaGamma)))
        return (deltaAlpha, deltaBeta, deltaGamma)
    
    def rapidMove(self, targetPosition):
        #Moves a set of limbs to a targetPosition (x,y)
        x,y,z=targetPosition

        #Calculates necessary bend angle for current iteration
        (deltaHip, deltaFemur, deltaTibia)=self.calculateHipFemurTibiaBend(targetPosition)
        
        #Checks if move is successful 
        if not self.bendLimbJoints(deltaHip, deltaFemur, deltaTibia):
            print('Error trying to bend limb. Hip: %s, Femur Bend: %s, Tibia Bend: %s' % (deltaHip, deltaFemur, deltaTibia))
            return False
        else:                       
            #Calculates current distance to target position
            currentDistance=self.distTo(x,y,z)
            ##self.limbs[limbIndex].printPosition()
            print('Target position reached. Current distance: %s' % currentDistance)
            ##print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltaHip, deltaFemur, deltaTibia, currentDistance))
            return True
        
    def linearMove(self, targetPosition, precision=0.5, maxAngleVar=10, maxIterations=500):
        #Moves limbs to a targetPosition (x,y)
        i=0
        x,y,z=targetPosition
        
        #Calculate current Distance to target Position
        currentDistance=self.distTo(x,y,z)
        
        lastMoveFine=True
        while currentDistance>precision and i<maxIterations and lastMoveFine:
            i+=1
            #Calculates necessary bend angle for current iteration
            (deltaHip, deltaFemur, deltaTibia)=self.iterateHipFemurTibiaBend(targetPosition, maxAngleVar)
            
            #Checks if move is successful 
            if not self.bendLimbJoints(deltaHip, deltaFemur, deltaTibia):
                print('Error trying to bend limb Hip: %s, Femur Bend: %s, Tibia Bend: %s' % ( deltaHip, deltaFemur, deltaTibia))
                lastMoveFine=False
            else:                       
                #Calculates current distance to target position
                currentDistance=self.distTo(x,y,z)
##                    self.printPosition()
##                    print('Distance:%s' % currentDistance)
##                    print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltaHip, deltaFemur, deltaTibia, currentDistance))

        if currentDistance<=precision:
            print('Target position reached. Current distance: %s Iterations: %s' % (currentDistance, i))
            return True
        else:
            print('Unable to reach target position. Current distance %s' % currentDistance)
            print('Iterations: %s' % i)
            if not lastMoveFine:
                print('Could not make a movement. Limits reached?')
            return False
        

    def updatePositions(self):
        self.setFemurAngle(self.femur.angle)
        self.setTibiaAngle(self.tibia.angle)

    def defaultPosition(self):
        self.rapidMove([120,0,-40])

    def stretch(self):
        #Stretchs the Limbs granting that it will do it in the air whitout touching the ground
        #TODO: Detect max femur and tibia angle permited to go
        if not self.femur.setAngleToMinOrMax(1):
            return False
        else:
            sleep(1)
        if not self.tibia.setAngleToMinOrMax(-1):
            return False
        else:
            sleep(1)
        if not self.setFemurAngle(0):
            return False
        else:
            sleep(1)
            return True
        
        
        
