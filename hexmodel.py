import logging
import json
import numpy as np
import time
from hexlimb import HexLimb
from hexengine import HexEngine

class HexModel(object):
    def __init__(self):
        limbstartposition = [120, 0, -30]

        self.calibrationfile = 'calibration.json'
        cal = self._loadcalibration()
        self.limbs = {
            1: HexLimb(0x40, [0, 1, 2], [27, 70, 123], [1, 1, 1], cal[1], [100, 80, -80]),
            2: HexLimb(0x40, [4, 5, 6], [27, 70, 123], [1, 1, 1], cal[2], [100, 0, -80]),
            3: HexLimb(0x40, [8, 9, 10], [27, 70, 123], [1, 1, 1], cal[3], [100, -80, -80]),
            4: HexLimb(0x60, [0, 1, 2], [27, 70, 123], [0, 0, 0], cal[4], [100, 80, -80]),
            5: HexLimb(0x60, [4, 5, 6], [27, 70, 123], [0, 0, 0], cal[5], [100, 0, -80]),
            6: HexLimb(0x60, [8, 9, 10], [27, 70, 123], [0, 0, 0], cal[6], [100, -80, -80]),
        }

        self.limbs[1].side = 'left'
        self.limbs[2].side = 'left'
        self.limbs[3].side = 'left'
        self.limbs[4].side = 'right'
        self.limbs[5].side = 'right'
        self.limbs[6].side = 'right'

        # Initializing the Engine:
        self.engine=HexEngine(self)

        print("HexModel initialized")

    def poweroff(self):
        for limbindex, limb in self.limbs.items():
            limb.poweroff()

    def poweron(self):
        for limbindex, limb in self.limbs.items():
            limb.poweron()

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
