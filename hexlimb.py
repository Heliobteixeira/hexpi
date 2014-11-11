import sys
##import numpy as np
from hexbone import HexBone
import math
from time import sleep


class HexLimb(object):
    ##Reformular com base nas alteracoes da classe Servo
    def __init__(self, I2C_ADDRESS, boneschannelsarray, boneslengtharray, revarray, offsetarray, origin):
        # TODO: Set limb starting position accordingly to init vars
        #0->Hip;1->Femur;2->Tibia

        ## Hexbone's min/max default angles
        defaultminangle = -90
        defaultmaxangle = +90

        #Setting i2c address and servo channels
        self.hip = HexBone(I2C_ADDRESS, boneschannelsarray[0], boneslengtharray[0], 0, 158, 643, revarray[0],
                           offsetarray[0], defaultminangle, defaultmaxangle, self.calcposition)
        self.femur = HexBone(I2C_ADDRESS, boneschannelsarray[1], boneslengtharray[1], 0, 149, 651, revarray[1],
                             offsetarray[1], defaultminangle, defaultmaxangle, self.calcposition)
        self.tibia = HexBone(I2C_ADDRESS, boneschannelsarray[2], boneslengtharray[2], 0, 158, 643, revarray[2],
                             offsetarray[2], defaultminangle, defaultmaxangle, self.calcposition)

        self.origin = [0, 0, 0]  #Correct this along the code and include as parameter of __init__

        self.x=None
        self.y=None
        self.z=None

    def __repr__(self):
        return ' Hip:{:> 6.1f} | Femur:{:> 6.1f} | Tibia:{:> 6.1f} | x={:> 6.1f} | y={:> 6.1f} | z={:> 6.1f} '.format(self.getnorhipangle(),
                                                                                             self.getnorfemurangle(),
                                                                                             self.getnortibiaangle(),
                                                                                             self.x, self.y, self.z)

    def poweroff(self):
        self.hip.poweroff()
        self.femur.poweroff()
        self.tibia.poweroff()

    def poweron(self):
        self.hip.poweron()
        self.femur.poweron()
        self.tibia.poweron()

    def getposition(self):
        return [self.x, self.y, self.z]

    ## Femur
    def setfemurangle(self, angle):
        return self.femur.setangle(angle)

    def checkfemurbend(self, bendangle):
        return self.femur.checkincangle(bendangle)

    def bendfemur(self, angle):
        return self.femur.incangle(angle)

    def getabsfemurangle(self):
        return self.femur.angle

    def getnorfemurangle(self):
        return self.femur.angle

    ## Tibia
    def settibiaangle(self, angle):
        return self.tibia.setangle(angle - 90)

    def checktibiabend(self, bendangle):
        return self.tibia.checkincangle(bendangle)

    def bendtibia(self, angle):
        return self.tibia.incangle(angle)

    def getabstibiaangle(self):
        return self.tibia.angle

    def getnortibiaangle(self):
        return 90 + self.tibia.angle

    ## Hip
    def sethipangle(self, angle):
        return self.hip.setangle(angle)

    def checkhipbend(self, bendangle):
        return self.hip.checkincangle(bendangle)

    def bendhip(self, angle):
        return self.hip.incangle(angle)

    def getabshipangle(self):
        return self.hip.angle

    def getnorhipangle(self):
        return self.hip.angle

    ## Hip + Femur + Tibia
    def setjoints(self, hipangle, femurangle, tibiaangle):
        #Sets Hip, Femur and Tibia angles simultaneously
        if self.sethipangle(hipangle) and self.setfemurangle(femurangle) and self.settibiaangle(
                tibiaangle):
            return True
        else:
            return False

    def bendjoints(self, hipbendangle, femurbendangle, tibiabendangle):
        #Bends Hip, Femur and Tibia simultaneously
        if self.checkhipbend(hipbendangle) and self.checkfemurbend(femurbendangle) and self.checktibiabend(
                tibiabendangle):
            self.bendhip(hipbendangle)
            self.bendfemur(femurbendangle)
            self.bendtibia(tibiabendangle)
            return True
        else:
            return False

    ## Offsets
    def setoffsets(self, hipoffset=None, femuroffset=None, tibiaoffset=None):
        success = True
        if hipoffset is not None:
            if not self.hip.setoffset(hipoffset):
                success = False

        if femuroffset is not None:
            if not self.femur.setoffset(femuroffset):
                success = False

        if tibiaoffset is not None:
            if not self.tibia.setoffset(tibiaoffset):
                success = False

        return success

    def incoffsets(self, hipoffset=None, femuroffset=None, tibiaoffset=None):
        success = True
        if hipoffset is not None:
            if not self.hip.incoffset(hipoffset):
                success = False

        if femuroffset is not None:
            if not self.femur.incoffset(femuroffset):
                success = False

        if tibiaoffset is not None:
            if not self.tibia.incoffset(tibiaoffset):
                success = False

        return success

    def calcposition(self):
        #Callback function (when setAngle is called) to update tip position
        try:
            L1 = self.hip.length
            L2 = self.femur.length
            L3 = self.tibia.length
            alpha = math.radians(self.getnorhipangle())
            beta = math.radians(self.getnorfemurangle())  # RADIANS!!!
            gamma = math.radians(self.getnortibiaangle())
            ##            self.x=L1*math.cos(math.radians(a1))+L2*math.cos(math.radians(a1-a2))
            ##            self.z=L1*math.sin(math.radians(a1))+L2*math.sin(math.radians(a1-a2))

            projL = L1 + L2 * math.cos(beta) + L3 * math.cos(beta - gamma)
            self.x = self.origin[0] + math.cos(alpha) * projL
            self.y = self.origin[1] + math.sin(alpha) * projL
            self.z = self.origin[2] + L2 * math.sin(beta) + L3 * math.sin(beta - gamma)
        except:
            return False
        else:
            return True

    def updatepositions(self):
        self.sethipangle(self.hip.angle)
        self.setfemurangle(self.femur.angle)
        self.settibiaangle(self.tibia.angle)

        
        
        
