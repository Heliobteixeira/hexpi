import sys
##import numpy as np
from hexbone import HexBone
from InverseKinematics import ik
import math
from time import sleep


class HexLimb(object):
    ##Reformular com base nas alteracoes da classe Servo
    def __init__(self, I2C_ADDRESS, boneschannelsarray, boneslengtharray, revarray, offsetarray, startposition):
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
        self.startposition = startposition
        #Reset servo positions after servo calibration
        self.x, self.y, self.z = 0, 0, 0
        self.defaultposition()
        self.calcposition()

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

    ##Code hip movement functions
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

    def printposition(self):
        print(' Hip: %2.1f   | Femur: %2.1f    | Tibia: %2.1f    |  x=%.2f;y=%.2f;z=%.2f' % (self.getnorhipangle(),
                                                                                             self.getnorfemurangle(),
                                                                                             self.getnortibiaangle(),
                                                                                             self.x, self.y, self.z))

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

    def bendlimbjoints(self, hipbendangle, femurbendangle, tibiabendangle):
        #Bends Femur and Tibia simultaneously
        if self.checkhipbend(hipbendangle) and self.checkfemurbend(femurbendangle) and self.checktibiabend(
                tibiabendangle):
            self.bendhip(hipbendangle)
            self.bendfemur(femurbendangle)
            self.bendtibia(tibiabendangle)
            return True
        else:
            return False

    def distto(self, x, y, z):
        return math.sqrt((x - self.x) ** 2 + (y - self.y) ** 2 + (z - self.z) ** 2)

    def iteratehipfemurtibiabend(self, targetPosition, maxAngleDisp):
        """

        :rtype : object
        """
        origin = [0, 0, 0]
        L1 = self.hip.length
        L2 = self.femur.length
        L3 = self.tibia.length
        alpha = math.radians(self.getnorhipangle())
        beta = math.radians(self.getnorfemurangle())
        gamma = math.radians(self.getnortibiaangle())
        (deltaHip, deltaFemur, deltaTibia) = ik.ikJacobian3DOF(L1, L2, L3, alpha, beta, gamma, origin, targetPosition)
        (deltaHip, deltaFemur, deltaTibia) = (
            math.degrees(deltaHip), math.degrees(deltaFemur), math.degrees(deltaTibia))

        currMaxAngle = max(abs(deltaHip), abs(deltaFemur), abs(deltaTibia))

        if currMaxAngle > maxAngleDisp:
            fact = maxAngleDisp / currMaxAngle
            (deltaHip, deltaFemur, deltaTibia) = (deltaHip * fact, deltaFemur * fact, deltaTibia * fact)

        ##print('Bend Angles: Hip: %s, Femur:%s, Tibia:%s' % (deltaHip, deltaFemur, deltaTibia))
        return (deltaHip, deltaFemur, deltaTibia)

    def calculatehipfemurtibiabend(self, targetposition):
        origin = [0, 0, 0]
        L1 = self.hip.length
        L2 = self.femur.length
        L3 = self.tibia.length

        ## Fetching current angles position:
        startAlpha = math.radians(self.getnorhipangle())
        startBeta = math.radians(self.getnorfemurangle())
        startGamma = math.radians(self.getnortibiaangle())

        ## Calculating final angles position:
        (endalpha, endbeta, endgamma) = ik.ikAnalytical3DOF(L1, L2, L3, origin, targetposition)

        deltaalpha = math.degrees(endalpha - startAlpha)
        deltabeta = math.degrees(endbeta - startBeta)
        deltagamma = math.degrees(endgamma - startGamma)

        #print('Will rotate:'+str((deltaalpha, deltabeta, deltaGamma)))
        return (deltaalpha, deltabeta, deltagamma)

    def rapidmove(self, targetposition):
        #Moves a set of limbs to a targetPosition (x,y)
        x, y, z = targetposition

        #Calculates necessary bend angle for current iteration
        (deltahip, deltafemur, deltatibia) = self.calculatehipfemurtibiabend(targetposition)

        #Checks if move is successful 
        if not self.bendlimbjoints(deltahip, deltafemur, deltatibia):
            print(
                'Error trying to bend limb. Hip: %s, Femur Bend: %s, Tibia Bend: %s' % (
                deltahip, deltafemur, deltatibia))
            return False
        else:
            #Calculates current distance to target position
            currentDistance = self.distto(x, y, z)
            ##self.limbs[limbIndex].printPosition()
            #print('Target position reached. Current distance: %s' % currentDistance)
            ##print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltahip, deltaFemur, deltatibia, currentDistance))
            return True

    def linearmove(self, targetposition, precision=0.5, maxanglevar=2, maxiterations=500):
        #Moves limbs to a targetPosition (x,y)
        i = 0
        x, y, z = targetposition

        #Calculate current Distance to target Position
        currentdistance = self.distto(x, y, z)

        lastmovefine = True
        while currentdistance > precision and i < maxiterations and lastmovefine:
            i += 1
            #Calculates necessary bend angle for current iteration
            (deltahip, deltafemur, deltatibia) = self.iteratehipfemurtibiabend(targetposition, maxanglevar)

            #Checks if move is successful 
            if not self.bendlimbjoints(deltahip, deltafemur, deltatibia):
                print('Error trying to bend limb Hip: %s, Femur Bend: %s, Tibia Bend: %s' % (
                    deltahip, deltafemur, deltatibia))
                lastmovefine = False
            else:
                #Calculates current distance to target position
                currentdistance = self.distto(x, y, z)
                ##                    self.printPosition()
                ##                    print('Distance:%s' % currentdistance)
                ##                    print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltahip, deltafemur, deltatibia, currentdistance))

        if currentdistance <= precision:
            #print('Target position reached. Current distance: %s Iterations: %s' % (currentdistance, i))
            return True
        else:
            print('Unable to reach target position. Current distance %s' % currentdistance)
            print('Iterations: %s' % i)
            if not lastmovefine:
                print('Could not make a movement. Limits reached?')
            return False


    def updatepositions(self):
        self.setfemurangle(self.femur.angle)
        self.settibiaangle(self.tibia.angle)

    def defaultposition(self):
        self.rapidmove(self.startposition)

    def stretch(self):
        #Stretchs the Limbs granting that it will do it in the air whitout touching the ground
        #TODO: Detect max femur and tibia angle permited to go
        if not self.femur.setangletominormax(1):
            return False
        else:
            sleep(1)
        if not self.tibia.setangletominormax(-1):
            return False
        else:
            sleep(1)
        if not self.setfemurangle(0):
            return False
        else:
            sleep(1)
            return True
        
        
        
