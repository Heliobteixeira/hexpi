import json
import numpy as np
import time
import hexlimb
import thread
import threading

class HexModel(object):
    def __init__(self):
        limbStartPosition=[120,0,-30]

        self.calibrationFile = 'calibration.json'
        cal=self._loadCalibration()
        self.limbs={
                     1: hexlimb.HexLimb(0x40, [0,1,2], [27,70,123], [1,1,1], cal[1], limbStartPosition),
		     2: hexlimb.HexLimb(0x40, [4,5,6], [27,70,123], [1,1,1], cal[2], limbStartPosition),
		     3: hexlimb.HexLimb(0x40, [8,9,10], [27,70,123], [1,1,1], cal[3], limbStartPosition),
		     4: hexlimb.HexLimb(0x60, [0,1,2], [27,70,123], [0,0,0], cal[4], limbStartPosition),
		     5: hexlimb.HexLimb(0x60, [4,5,6], [27,70,123], [0,0,0], cal[5], limbStartPosition),
		     6: hexlimb.HexLimb(0x60, [8,9,10], [27,70,123], [0,0,0], cal[6], limbStartPosition),
                    }

        self.limbs[1].side='left'
        self.limbs[2].side='left'
        self.limbs[3].side='left'
        self.limbs[4].side='right'
        self.limbs[5].side='right'
        self.limbs[6].side='right'
        
        
        self.precision=0.5
        self.LAMBDA=200  # 20<LAMBDA<50
        self.maxIterations=500
        self.position=[0,0,0]
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

    def startThreadsArray(self, threadsArray):
        for thread in threadsArray:
            thread.start()

    def waitForThreadsArray(self, threadsArray):
        for thread in threadsArray:
            thread.join()                

    def sumVectors(self, vector1, vector2):
        v1=np.array(vector1)
        v2=np.array(vector2)
        return v1+v2
        
    def tripodGait(self, dispVector, maxAngleVar=1):
        dispVector=np.array(dispVector)
        halfDispVector=dispVector/2.0
        clearanceVector=np.array([0,0,30])

        tripod1=[1,3,5]
        tripod2=[2,4,6]
        
        try:
            # Gait Sequence
            self.moveLimbs(tripod1, (halfDispVector+clearanceVector), False) #Lift limbs 1,3,5
            #time.sleep(2)
            self.moveLimbs(tripod1, (halfDispVector-clearanceVector), False)
            #time.sleep(2)
            self.moveLimbs(tripod2, (halfDispVector+clearanceVector), False)
            #time.sleep(2)
            self.moveLimbs(tripod1, (-dispVector), True)
            #time.sleep(2)
            self.moveLimbs(tripod2, (halfDispVector-clearanceVector), False)
            #time.sleep(2)
            self.moveLimbs(tripod1, (clearanceVector), False)
            #time.sleep(2)
            self.moveLimbs(tripod2, (-dispVector), True)
            #time.sleep(2)
            self.moveLimbs(tripod1, (-clearanceVector), False)
            #time.sleep(2)
            
        finally:
            #Reposition all limbs in starting position
            print('Reposition all limbs in starting position')
            

    def moveLimbs(self, arrayLimbs, dispVector, interpMove=True, maxAngleVar=3, precision=0.5):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads=[]
        #Iterates ALL limbs to approximate desired target position
        for limbIndex in arrayLimbs:
            limb=self.limbs[limbIndex]
            limbDispVector=list(dispVector) #Copy by value
            #print('limbDispVector:',limbDispVector)
            if limb.side=='left':
                limbDispVector[0]=-limbDispVector[0]
            #print('Limb#%s (%s), moving to: %s',(limbIndex, limb.side, limbDispVector))    
            limbTargetPosition=self.sumVectors(limb.getPosition(),limbDispVector)
            if interpMove:
                threads.append(self.linearMoveLimbT(limbIndex, limb, limbTargetPosition, precision, maxAngleVar))
            else:
                threads.append(self.rapidMoveLimbT(limbIndex, limb, limbTargetPosition))
                    
        self.startThreadsArray(threads)
        self.waitForThreadsArray(threads)

    def positionLimbs(self, arrayLimbs, targetPosition, interpMove=True, maxAngleVar=2, precision=0.5):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads=[]
        #Iterates ALL limbs to approximate desired target position
        for limbIndex in arrayLimbs:
            limb=self.limbs[limbIndex]
            if interpMove:
                threads.append(self.linearMoveLimbT(limbIndex, limb, targetPosition, precision, maxAngleVar))
            else:
                threads.append(self.rapidMoveLimbT(limbIndex, limb, targetPosition))
                
        self.startThreadsArray(threads)
        self.waitForThreadsArray(threads)
        
    def printLimbsPosition(self):
        for limbIndex, limb in self.limbs.items():
            coord=limb.getPosition()
            print('Limb #%s x=%3.1f y=%3.1f z=%3.1f' % (limbIndex, coord[0], coord[1], coord[2]))


    def setLimbOffsets(self, limbIndex, hipOffset=None, femurOffset=None, tibiaOffset=None ):
        if limbIndex not in self.limbs.keys():
            print('Invalid limbIndex:%s' % limbIndex)
            return False
        
        if not self.limbs[limbIndex].setOffsets(hipOffset,femurOffset,tibiaOffset):
            print('Error applying offset values to limbIndex:%s' % limbIndex)

        #Saves current offset values
        self._saveCalibration()
  
    def incLimbOffsets(self, limbIndex, hipOffset=None, femurOffset=None, tibiaOffset=None ):
        if limbIndex not in self.limbs.keys():
            print('Invalid limbIndex:%s' % limbIndex)
            return False
        
        if not self.limbs[limbIndex].incOffsets(hipOffset,femurOffset,tibiaOffset):
            print('Error incrementing offset values to limbIndex:%s' % limbIndex)

        #Saves current offset values
        self._saveCalibration()

    def _loadCalibration(self):
        calibration={}
        try:
            file=open(self.calibrationFile, 'r')
            limbsCalibration=json.load(file)
        except IOError:
            pass
        else:
            print('Loading calibration file: %s' % self.calibrationFile)
            for limbIndex, limb in limbsCalibration.iteritems():
                calibration[int(limbIndex)]=([int(limb['hip']),int(limb['femur']),int(limb['tibia'])])

        return calibration

    def _loadAndApplyCalibration(self):
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
