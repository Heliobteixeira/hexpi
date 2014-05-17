import json
import numpy as np
import time
import hexlimb
import thread
import threading

class HexModel(object):
    def __init__(self):
        limbStartPosition=[120,0,-30]
        self.limbs={
                     1: hexlimb.HexLimb(0x40, [0,1,2], [25,70,123], [1,1,1], limbStartPosition),
		     2: hexlimb.HexLimb(0x40, [4,5,6], [25,70,123], [1,1,1], limbStartPosition),
		     3: hexlimb.HexLimb(0x40, [8,9,10], [25,70,123], [1,1,1], limbStartPosition),
		     4: hexlimb.HexLimb(0x60, [0,1,2], [25,70,123], [0,0,0], limbStartPosition),
		     5: hexlimb.HexLimb(0x60, [4,5,6], [25,70,123], [0,0,0], limbStartPosition),
		     6: hexlimb.HexLimb(0x60, [8,9,10], [25,70,123], [0,0,0], limbStartPosition),
                    }
        
        self.calibrationFile = 'calibration.json'
        self._loadCalibration()
        self.precision=0.5
        self.LAMBDA=200  # 20<LAMBDA<50
        self.maxIterations=500
        print("HexModel initialized")

    class rapidMoveLimbT (threading.Thread):
        def __init__(self, threadID, limb, targetPosition):
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.limb = limb
            self.targetPosition = targetPosition
        def run(self):
            self.limb.rapidMove(self.targetPosition)   

    class linearMoveLimbT (threading.Thread):
        def __init__(self, threadID, limb, targetPosition, precision, maxAngleVar):
            threading.Thread.__init__(self)
            self.threadID = threadID
            self.limb = limb
            self.targetPosition = targetPosition
            self.precision = precision
            self.maxAngleVar = maxAngleVar
        def run(self):
            self.limb.linearMove(self.targetPosition, self.precision, self.maxAngleVar, 200) 

    def powerOff(self):
        for limbIndex, limb in self.limbs.items():
            limb.powerOff()
            
    def powerOn(self):
        for limbIndex, limb in self.limbs.items():
            limb.powerOn()
            
    def stepLimbsSequence(self, indexSequenceArray, wait=1):
        for limbIndex in indexSequenceArray:
            self.limbs[limbIndex].doStep(wait)

    def stretchLimbs(self):
        for limbIndex, limb in self.limbs.items():
            if not limb.stretch():
                return False
        return True

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

    def linearMoveLimbs(self, arrayLimbs, targetPosition, precision=0.5, maxAngleVar=10):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads=[]
        #Iterates ALL limbs to approximate desired target position
        for limbIndex, limb in self.limbs.items():
            threads.append(self.linearMoveLimbT(limbIndex, limb, targetPosition, precision, maxAngleVar))

        for thread in threads:
            thread.start()

    def rapidMoveLimbs(self, arrayLimbs, targetPosition):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads=[]
        #Iterates ALL limbs to approximate desired target position
        for limbIndex, limb in self.limbs.items():
            threads.append(self.rapidMoveLimbT(limbIndex, limb, targetPosition))

        for thread in threads:
            thread.start()
        

    def setLimbOffsets(self, limbIndex, hipOffset=None, femurOffset=None, tibiaOffset=None ):
        if limbIndex not in self.limbs.keys():
            print('Invalid limbIndex:%s' % limbIndex)
            return False
        
        if not self.limbs[limbIndex].setOffsets(hipOffset,femurOffset,tibiaOffset):
            print('Error applying offset values to limbIndex:%s' % limbIndex)

        #Saves current offset values
        self._saveCalibration()
  

    def _loadCalibration(self):
        try:
            file=open(self.calibrationFile, 'r')
            limbsCalibration=json.load(file)
        except IOError:
            pass
        else:
            print('Loading calibration file: %s' % self.calibrationFile)
            for limbIndex, limb in limbsCalibration.iteritems():
                self.limbs[int(limbIndex)].setOffsets(int(limb['hip']), int(limb['femur']), int(limb['tibia']))

    def _saveCalibration(self):
        #Saves current offset value for set for each servo composing the limbs
        limbsCalibration={}
        for limbIndex, limb in self.limbs.items():
            auxCalib={'hip':limb.hip.offset, 'femur':limb.femur.offset, 'tibia':limb.tibia.offset}
            limbsCalibration[limbIndex]=auxCalib

        
        with open(self.calibrationFile, 'w') as file:
            json.dump(limbsCalibration, file)
