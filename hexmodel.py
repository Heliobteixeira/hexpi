import json
import numpy as np
import time
import hexlimb
import thread
import threading


class HexModel(object):
    def __init__(self):
        limbstartposition = [120, 0, -30]

        self.calibrationfile = 'calibration.json'
        cal = self._loadcalibration()
        self.limbs = {
            1: hexlimb.HexLimb(0x40, [0, 1, 2], [27, 70, 123], [1, 1, 1], cal[1], limbstartposition),
            2: hexlimb.HexLimb(0x40, [4, 5, 6], [27, 70, 123], [1, 1, 1], cal[2], limbstartposition),
            3: hexlimb.HexLimb(0x40, [8, 9, 10], [27, 70, 123], [1, 1, 1], cal[3], limbstartposition),
            4: hexlimb.HexLimb(0x60, [0, 1, 2], [27, 70, 123], [0, 0, 0], cal[4], limbstartposition),
            5: hexlimb.HexLimb(0x60, [4, 5, 6], [27, 70, 123], [0, 0, 0], cal[5], limbstartposition),
            6: hexlimb.HexLimb(0x60, [8, 9, 10], [27, 70, 123], [0, 0, 0], cal[6], limbstartposition),
        }

        self.limbs[1].side = 'left'
        self.limbs[2].side = 'left'
        self.limbs[3].side = 'left'
        self.limbs[4].side = 'right'
        self.limbs[5].side = 'right'
        self.limbs[6].side = 'right'

        self.precision = 0.5
        self.LAMBDA = 200  # 20<LAMBDA<50
        self.maxIterations = 500
        self.position = [0, 0, 0]
        print("HexModel initialized")

    class RapidMoveLimbT(threading.Thread):
        def __init__(self, threadid, limb, targetposition):
            threading.Thread.__init__(self)
            self.threadid = threadid
            self.limb = limb
            self.targetposition = targetposition

        def run(self):
            self.limb.rapidmove(self.targetposition)

    class LinearMoveLimbT(threading.Thread):
        def __init__(self, threadid, limb, targetposition, precision, maxanglevar):
            threading.Thread.__init__(self)
            self.threadid = threadid
            self.limb = limb
            self.targetposition = targetposition
            self.precision = precision
            self.maxanglevar = maxanglevar

        def run(self):
            self.limb.linearmove(self.targetposition, self.precision, self.maxanglevar, 200)

    def poweroff(self):
        for limbindex, limb in self.limbs.items():
            limb.poweroff()

    def poweron(self):
        for limbindex, limb in self.limbs.items():
            limb.poweron()

    def steplimbssequence(self, indexSequenceArray, wait=1):
        for limbindex in indexSequenceArray:
            self.limbs[limbindex].doStep(wait)

    def stretchlimbs(self):
        for limbindex, limb in self.limbs.items():
            if not limb.stretch():
                return False
        return True

    def standposition(self):
        for limbindex, limb in self.limbs.items():
            limb.defaultposition()

    def steplimbindex(self, limbindex, wait=1):
        self.limbs[limbindex].bendfemur(60)
        self.limbs[limbindex].bendhip(60)
        time.sleep(wait)
        self.limbs[limbindex].bendfemur(-60)
        time.sleep(wait)
        self.limbs[limbindex].bendhip(-60)
        time.sleep(wait)

    def startthreadsarray(self, threadsarray):
        for thread in threadsarray:
            thread.start()

    def waitforthreadsarray(self, threadsarray):
        for thread in threadsarray:
            thread.join()

    def sumvectors(self, vector1, vector2):
        v1 = np.array(vector1)
        v2 = np.array(vector2)
        return v1 + v2

    def tripodgait(self, dispvector, maxanglevar=1):
        dispvector = np.array(dispvector)
        halfdispvector = dispvector / 2.0
        clearancevector = np.array([0, 0, 30])

        tripod1 = [1, 3, 5]
        tripod2 = [2, 4, 6]

        try:
            # Gait Sequence
            self.movelimbs(tripod1, (halfdispvector + clearancevector), False)  #Lift limbs 1,3,5
            #time.sleep(2)
            self.movelimbs(tripod1, (halfdispvector - clearancevector), False)
            #time.sleep(2)
            self.movelimbs(tripod2, (halfdispvector + clearancevector), False)
            #time.sleep(2)
            self.movelimbs(tripod1, (-dispvector), True)
            #time.sleep(2)
            self.movelimbs(tripod2, (halfdispvector - clearancevector), False)
            #time.sleep(2)
            self.movelimbs(tripod1, (clearancevector), False)
            #time.sleep(2)
            self.movelimbs(tripod2, (-dispvector), True)
            #time.sleep(2)
            self.movelimbs(tripod1, (-clearancevector), False)
            #time.sleep(2)

        finally:
            #Reposition all limbs in starting position
            print('Reposition all limbs in starting position')


    def movelimbs(self, arraylimbs, dispvector, interpmove=True, maxanglevar=3, precision=0.5):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads = []
        #Iterates ALL limbs to approximate desired target position
        for limbindex in arraylimbs:
            limb = self.limbs[limbindex]
            limbdispvector = list(dispvector)  #Copy by value
            #print('limbdispvector:',limbdispvector)
            if limb.side == 'left':
                limbdispvector[0] = -limbdispvector[0]
            #print('Limb#%s (%s), moving to: %s',(limbindex, limb.side, limbdispvector))
            limbtargetposition = self.sumvectors(limb.getposition(), limbdispvector)
            if interpmove:
                threads.append(self.LinearMoveLimbT(limbindex, limb, limbtargetposition, precision, maxanglevar))
            else:
                threads.append(self.RapidMoveLimbT(limbindex, limb, limbtargetposition))

        self.startthreadsarray(threads)
        self.waitforthreadsarray(threads)

    def positionlimbs(self, arraylimbs, targetposition, interpmove=True, maxanglevar=2, precision=0.5):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads = []
        #Iterates ALL limbs to approximate desired target position
        for limbindex in arraylimbs:
            limb = self.limbs[limbindex]
            if interpmove:
                threads.append(self.LinearMoveLimbT(limbindex, limb, targetposition, precision, maxanglevar))
            else:
                threads.append(self.RapidMoveLimbT(limbindex, limb, targetposition))

        self.startthreadsarray(threads)
        self.waitforthreadsarray(threads)

    def printlimbsposition(self):
        for limbindex, limb in self.limbs.items():
            coord = limb.getposition()
            print('Limb #%s x=%3.1f y=%3.1f z=%3.1f' % (limbindex, coord[0], coord[1], coord[2]))


    def setlimboffsets(self, limbindex, hipOffset=None, femurOffset=None, tibiaOffset=None):
        if limbindex not in self.limbs.keys():
            print('Invalid limbIndex:%s' % limbindex)
            return False

        if not self.limbs[limbindex].setoffsets(hipOffset, femurOffset, tibiaOffset):
            print('Error applying offset values to limbIndex:%s' % limbindex)

        #Saves current offset values
        self._savecalibration()

    def inclimboffsets(self, limbindex, hipOffset=None, femurOffset=None, tibiaOffset=None):
        if limbindex not in self.limbs.keys():
            print('Invalid limbIndex:%s' % limbindex)
            return False

        if not self.limbs[limbindex].incoffsets(hipOffset, femurOffset, tibiaOffset):
            print('Error incrementing offset values to limbIndex:%s' % limbindex)

        #Saves current offset values
        self._savecalibration()

    def _loadcalibration(self):
        calibration = {}
        try:
            file = open(self.calibrationfile, 'r')
            limbscalibration = json.load(file)
        except IOError:
            pass
        else:
            print('Loading calibration file: %s' % self.calibrationfile)
            for limbindex, limb in limbscalibration.iteritems():
                calibration[int(limbindex)] = ([int(limb['hip']), int(limb['femur']), int(limb['tibia'])])

        return calibration

    def _loadandapplycalibration(self):
        try:
            file = open(self.calibrationfile, 'r')
            limbscalibration = json.load(file)
        except IOError:
            pass
        else:
            print('Loading calibration file: %s' % self.calibrationfile)
            for limbIndex, limb in limbscalibration.iteritems():
                self.limbs[int(limbIndex)].setoffsets(int(limb['hip']), int(limb['femur']), int(limb['tibia']))

    def _savecalibration(self):
        #Saves current offset value for set for each servo composing the limbs
        limbscalibration = {}
        for limbindex, limb in self.limbs.items():
            auxcalib = {'hip': limb.hip.offset, 'femur': limb.femur.offset, 'tibia': limb.tibia.offset}
            limbscalibration[limbindex] = auxcalib

        with open(self.calibrationfile, 'w') as file:
            json.dump(limbscalibration, file)
