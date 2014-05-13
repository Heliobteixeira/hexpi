import json
import numpy as np
import time
import hexlimb

class HexModel(object):
    def __init__(self):
        self.limbs={
                     1: hexlimb.HexLimb(0x40, [0,1,2], [25,70,123], [1,1,1]),
		     2: hexlimb.HexLimb(0x40, [4,5,6], [25,70,123], [1,1,1]),
		     3: hexlimb.HexLimb(0x40, [8,9,10], [25,70,123], [1,1,1]),
		     4: hexlimb.HexLimb(0x60, [0,1,2], [25,70,123], [0,0,0]),
		     5: hexlimb.HexLimb(0x60, [4,5,6], [25,70,123], [0,0,0]),
		     6: hexlimb.HexLimb(0x60, [8,9,10], [25,70,123], [0,0,0]),
                    }
        
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()
        self.precision=0.5
        self.LAMBDA=200  # 20<LAMBDA<50
        self.maxIterations=500
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

    def moveLimbsTipTo(self, arrayLimbs, targetPosition):
        #Moves a set of limbs to a targetPosition (x,y)
        i=0
        x,y,z=targetPosition
        
        #Calculate current Distance to target Position
        maxLimbsDistance=0
        for limbIndex in arrayLimbs:
            maxLimbsDistance=max(maxLimbsDistance, self.limbs[limbIndex].distTo(x,y,z))
        currentDistance=maxLimbsDistance
        
        lastMoveFine=True
        while currentDistance>self.precision and i<self.maxIterations and lastMoveFine:
            i+=1
            maxLimbsDistance=0
            #Iterates ALL limbs to approximate desired target position
            for limbIndex in arrayLimbs:
                #Calculates necessary bend angle for current iteration
                (deltaHip, deltaFemur, deltaTibia)=self.limbs[limbIndex].iterateHipFemurTibiaBend(targetPosition, self.LAMBDA)
                
                #Checks if move is successful 
                if not self.limbs[limbIndex].bendLimbJoints(deltaHip, deltaFemur, deltaTibia):
                    print('Error trying to bend limb %s. Hip: %s, Femur Bend: %s, Tibia Bend: %s' % (limbIndex, deltaHip, deltaFemur, deltaTibia))
                    lastMoveFine=False
                else:                       
                    #Calculates current distance to target position
                    currentDistance=self.limbs[limbIndex].distTo(x,y,z)
                    maxLimbsDistance=max(maxLimbsDistance,currentDistance)
                    self.limbs[limbIndex].printPosition()
                    print('Distance:%s' % currentDistance)
                    ##print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltaHip, deltaFemur, deltaTibia, currentDistance))
            currentDistance=maxLimbsDistance

        if currentDistance<=self.precision:
            print('Target position reached. Current distance: %s Iterations: %s' % (currentDistance, i))
        else:
            print('Unable to reach target position. Current distance %s' % currentDistance)
            print('Iterations: %s' % i)
            if not lastMoveFine:
                print('Could not make a movement. Limits reached?')					

    def setLimbOffsets(self, limbIndex, hipOffset=None, femurOffset=None, tibiaOffset=None ):
        if limbIndex not in self.limbs.keys():
            print('Invalid limbIndex:%s' % limbIndex)
            return False
        
        if not self.limbs[limbIndex].setOffsets(hipOffset,femurOffset,tibiaOffset):
            print('Error applying offset values to limbIndex:%s' % limbIndex)

        #Saves current offset values
        self._saveCalibration()
  

    def _loadCalibration(self):
        ##TODO
        try:
            file=open(self.calibrationFile, 'r')
            limbsCalibration=json.load(file)
        except IOError:
            pass
        else:
            for limbIndex, limb in limbsCalibration.iteritems():
                print(limb)
                self.limbs[int(limbIndex)].hip.offset=int(limb['hip'])
                self.limbs[int(limbIndex)].femur.offset=int(limb['femur'])
                self.limbs[int(limbIndex)].tibia.offset=int(limb['tibia'])

    def _saveCalibration(self):
        #Saves current offset value for set for each servo composing the limbs
        limbsCalibration={}
        for limbIndex, limb in self.limbs.items():
            auxCalib={'hip':limb.hip.offset, 'femur':limb.femur.offset, 'tibia':limb.tibia.offset}
            limbsCalibration[limbIndex]=auxCalib

        
        with open(self.calibrationFile, 'w') as file:
            json.dump(limbsCalibration, file)
