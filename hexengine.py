import logging
import numpy as np
import time
import threading
import math
import sys
import pdb
from copy import copy
from Kinematics import ik

import pprint

class CyclicList(list):
    def lifoappend(self, item):
        super(CyclicList, self).insert(0, item)
        super(CyclicList, self).pop()
    def setindexzero(self, index):
        for i in range(0, index):
            self.lifoappend(self[-1])


class HexEngine(object):

    def __init__(self, hexmodel):
        self.hexmodel=hexmodel
        self.precision = 0.5
        self.LAMBDA = 200  # 20<LAMBDA<50
        self.maxIterations = 500
        self.groundz = -30

        # Hip angles for stable position
        self.standhipangles = { 1:45,
                                2:0,
                                3:-45,
                                4:45,
                                5:0,
                                6:-45,
        }

        

        # Limbs Angles Set List
        self.limbslastpositions = {}
        self.limbsgaitpositions = {}
        self.nbrgaitsteps = 0 # Total number of calculated gait steps
        self.gaiton = False
        self.gaitproc = None

        self.standup()
        self.powered = True

    class RapidMoveLimbT(threading.Thread):
        def __init__(self, threadid, rapidmove, limb, targetposition):
            threading.Thread.__init__(self)
            self.threadid = threadid
            self.limb = limb
            self.targetposition = targetposition
            self.rapidmove=rapidmove

        def run(self):
            self.rapidmove(self.limb, self.targetposition)


    class LinearMoveLimbT(threading.Thread):
        def __init__(self, threadid, linearmove, limb, targetposition, precision, maxanglevar):
            threading.Thread.__init__(self)
            self.threadid = threadid
            self.limb = limb
            self.targetposition = targetposition
            self.precision = precision
            self.maxanglevar = maxanglevar
            self.linearmove=linearmove

        def run(self):
            self.linearmove(self.limb, self.targetposition, self.precision, self.maxanglevar, 200)

    def steplimbssequence(self, indexSequenceArray, wait=1):
        for limbindex in indexSequenceArray:
            self.hexmodel.limbs[limbindex].doStep(wait)


    def stretchlimbs(self):
        for limbindex, limb in self.hexmodel.limbs.items():
            if not limb.stretch():
                return False
        return True


    def steplimbindex(self, limbindex, wait=1):
        self.hexmodel.limbs[limbindex].bendfemur(60)
        self.hexmodel.limbs[limbindex].bendhip(60)
        time.sleep(wait)
        self.hexmodel.limbs[limbindex].bendfemur(-60)
        time.sleep(wait)
        self.hexmodel.limbs[limbindex].bendhip(-60)
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

    def distto(self, limb, x, y, z):
        return math.sqrt((x - limb.x) ** 2 + (y - limb.y) ** 2 + (z - limb.z) ** 2)

    def midpoint(self, p0, p1):
        if not len(p0)==len(p1):
            sys.exit("Error trying to find middle point of two non dimensionally equally points")

        p0=np.array(p0) 
        p1=np.array(p1)

        return list((p0+p1)/2)

    def tripodgait(self, dispvector, maxanglevar=1):
        # This function is obsolete, and will be refactored. 
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


    def movelimbs(self, arraylimbs, dispvector, interpmove=True, maxanglevar=1, precision=0.5):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads = []
        #Iterates ALL limbs to approximate desired target position
        for limbindex in arraylimbs:
            limb = self.hexmodel.limbs[limbindex]
            limbdispvector = list(dispvector)  # Copy by value
            #print('limbdispvector:',limbdispvector)
            if limb.side == 'left':
                limbdispvector[0] = -limbdispvector[0] 
            #print('Limb#%s (%s), moving to: %s',(limbindex, limb.side, limbdispvector))
            limbtargetposition = self.sumvectors(limb.getposition(), limbdispvector)
            if interpmove:
                threads.append(self.LinearMoveLimbT(limbindex, self.linearmove, limb, limbtargetposition, precision, maxanglevar))
            else:
                threads.append(self.RapidMoveLimbT(limbindex, self.rapidmove, limb, limbtargetposition))

        self.startthreadsarray(threads)
        self.waitforthreadsarray(threads)

    def positionlimbs(self, arraylimbs, targetposition, interpmove=True, maxanglevar=2, precision=0.5):
        #Moves a set of limbs to a targetPosition (x,y,z)
        threads = []
        #Iterates ALL limbs to approximate desired target position
        for limbindex in arraylimbs:
            limb = self.hexmodel.limbs[limbindex]
            if interpmove:
                threads.append(self.LinearMoveLimbT(limbindex, self.linearmove, limb, targetposition, precision, maxanglevar))
            else:
                threads.append(self.RapidMoveLimbT(limbindex, self.rapidmove, limb, targetposition))

        self.startthreadsarray(threads)
        self.waitforthreadsarray(threads)

    def limbdistanceto(self, limb, x, y, z):
        return math.sqrt((x - limb.x) ** 2 + (y - limb.y) ** 2 + (z - limb.z) ** 2)

    def iteratehipfemurtibiabend(self, limb, targetPosition, maxAngleDisp):
    	# Returns a list of several angle duples representing the incremental angle displacements
        origin = [0, 0, 0]
        L1 = limb.hip.length
        L2 = limb.femur.length
        L3 = limb.tibia.length
        alpha = math.radians(limb.getnorhipangle())
        beta = math.radians(limb.getnorfemurangle())
        gamma = math.radians(limb.getnortibiaangle())

        maxAngleDisp=math.radians(maxAngleDisp)

        listofangularinc = ik.ikJacobian3DOF(L1, L2, L3, alpha, beta, gamma, origin, targetPosition, maxAngleDisp, 0.5, 100)

        ##print('Bend Angles: Hip: %s, Femur:%s, Tibia:%s' % (deltaHip, deltaFemur, deltaTibia))
        return listofangularinc

    def calculatehipfemurtibiabend(self, limb, targetposition):
        origin = [0, 0, 0]
        L1 = limb.hip.length
        L2 = limb.femur.length
        L3 = limb.tibia.length

        ## Fetching current angles position:
        startAlpha = math.radians(limb.getnorhipangle())
        startBeta = math.radians(limb.getnorfemurangle())
        startGamma = math.radians(limb.getnortibiaangle())

        ## Calculating final angles position:
        (endalpha, endbeta, endgamma) = ik.ikAnalytical3DOF(L1, L2, L3, origin, targetposition)

        deltaalpha = math.degrees(endalpha - startAlpha)
        deltabeta = math.degrees(endbeta - startBeta)
        deltagamma = math.degrees(endgamma - startGamma)

        #print('Will rotate:'+str((deltaalpha, deltabeta, deltaGamma)))
        return (deltaalpha, deltabeta, deltagamma)

    def rapidmove(self, limb, targetposition):
        #Moves a set of limbs to a targetPosition (x,y)
        x, y, z = targetposition

        #Calculates necessary bend angle for current iteration
        (deltahip, deltafemur, deltatibia) = self.calculatehipfemurtibiabend(limb, targetposition)

        #Checks if move is successful 
        if not limb.bendjoints(deltahip, deltafemur, deltatibia):
            print(
                'Error trying to bend limb. Hip: %s, Femur Bend: %s, Tibia Bend: %s' % (
                deltahip, deltafemur, deltatibia))
            return False
        else:
            #Calculates current distance to target position
            currentDistance = self.distto(limb, x, y, z)
            ##self.limbs[limbIndex].printPosition()
            #print('Target position reached. Current distance: %s' % currentDistance)
            ##print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltahip, deltaFemur, deltatibia, currentDistance))
            return True

    def linearmove(self, limb, targetposition, precision=0.5, maxanglevar=1, maxiterations=500):
        #Moves limbs to a targetPosition (x,y) using bendjoints
        i = 0
        x, y, z = targetposition

        #Calculate current Distance to target Position
        currentdistance = self.distto(limb, x, y, z)

        listofangularinc=self.iteratehipfemurtibiabend(limb, targetposition, maxanglevar)

        lastmovefine = True
        #print('Nbr of items:', len(listofangularinc))
        for angularinc in listofangularinc:
            i += 1
            #Calculates necessary bend angle for current iteration
            (deltahip, deltafemur, deltatibia) = map(math.degrees, angularinc)
##
            #Checks if move is successful 
            if not limb.bendjoints(deltahip, deltafemur, deltatibia):
                print('Error trying to bend limb Hip: %s, Femur Bend: %s, Tibia Bend: %s' % (
                    deltahip, deltafemur, deltatibia))
                lastmovefine = False
                break
            else:
                #Calculates current distance to target position
                currentdistance = self.distto(limb, x, y, z)
                #print('CurrentDistance:',currentdistance)
                continue ## ! Gotos directly to the begining of loop
##		        ## limb.printPosition()
##		        ## print('Distance:%s' % currentdistance)
##		        ## print('Hip;Femur;Tibia Bend: %s;%s;%s Distance: %s' % (deltahip, deltafemur, deltatibia, currentdistance))
##
        if currentdistance <= precision:
            #print('Target position reached. Current distance: %s Iterations: %s' % (currentdistance, i))
            return True
        else:
            print('Unable to reach target position. Current distance %s' % currentdistance)
            print('Iterations: %s' % i)
            if not lastmovefine:
                print('Could not make a movement. Limits reached?')
            return False

#    def startgait(self):
        
    def quadraticmove(self, limb, p1, p2, nbrsteps=50, delay=0):
        """ 
        Moves limb along a quadratic bezier curve 

        Keyword arguments:
        limb - of type HexLimb
        p1 - middle point of bezier curve
        p2 - end point of bezier curve
        nbrsteps - total number of points of curve
        delay - interval in seconds between each incremental move 
        """
        coxalength=limb.hip.length
        femurlength=limb.femur.length
        tibialength=limb.tibia.length
        origin=limb.origin
        p0=limb.getposition()
        
        limbsgaitpositions=[]
        limbsgaitpositions=ik.generatequadraticpath(coxalength, femurlength, tibialength, origin, p0, p1, p2, nbrsteps)
        #pdb.set_trace()
        for positionrow in limbsgaitpositions:
            (jointsangles, position)=positionrow
            (coxaangle, femurangle, tibiaangle)=jointsangles
            if not limb.setjoints(coxaangle, femurangle, tibiaangle):
                print('Unable to perform quadraticmove from %s>%s>%s' % (p0,p1,p2))
            time.sleep(delay)

    def standup(self):
        print('Starting standup...')
        # Raise femurs
        for limbindex, limb in self.hexmodel.limbs.items():
            limb.setfemurangle(85)
        time.sleep(0.5) # Wait for servo to reach its position
        #pdb.set_trace()
        # Retract tibias
        for limbindex, limb in self.hexmodel.limbs.items():
            limb.settibiaangle(145)
        time.sleep(0.5) # Wait for servo to reach its position
        #pdb.set_trace()
        # Position hips at stable angles
        for limbindex, limb in self.hexmodel.limbs.items():
            limb.sethipangle(self.standhipangles[limbindex])
        time.sleep(0.5)

        self.movelimbs([1,2,3,4,5,6], [0,0,-80], interpmove=True, maxanglevar=1, precision=0.5)
        print('...Finished standup')


    """
    def stretch(self):
        #Stretchs the Limbs granting that it will do it in the air whitout touching the ground
        #TODO: Detect max femur and tibia angle permited to go
        if not self.femur.setangletominormax(1):
            return False
        else:
            time.sleep(1)
        if not self.tibia.setangletominormax(-1):
            return False
        else:
            time.sleep(1)
        if not self.setfemurangle(0):
            return False
        else:
            time.sleep(1)
            return True
    """
    def updategait(self, delay):
        # Call this method in regular intervals to update gait
        limbsgaitpositions = copy(self.limbsgaitpositions)
        nbrgaitsteps = copy(self.nbrgaitsteps)
        gaitstep = 0
        
        while True:
            try:
                if self.gaiton:
                    if gaitstep>=nbrgaitsteps-1:
                        #print('New cycle')
                        gaitstep=0
                    else:
                        gaitstep+=1

                    for limbindex, limb in self.hexmodel.limbs.items():
                        (jointsangles, position)=limbsgaitpositions[limbindex][gaitstep]
                        (coxaangle, femurangle, tibiaangle)=jointsangles
                        limb.setjoints(coxaangle, femurangle, tibiaangle)
                    time.sleep(delay)    
                else:
                    print('Gait stopped')
                    return True
            except:
                sys.exit("Exception in updategait()")
                return False

    def loadwavegaitpaths (self, gaitvector, seq=(1,2,3,4,5,6), liftvector=(0,0,30), phasenbrsteps=10):
        """Loads global variable with the gait points for each limb
        
        Keyword arguments:
        gaitvector -- vector path followed by hex, each cycle
        seq        -- sequence of limbs movement
        liftvector -- vector path when raising limbs
        phasenbrsteps -- total number of steps = 6 x phasenbrsteps
                         greater phasenbrsteps equals greater precision and less speed
        """
        if type(gaitvector) is not tuple:
            print "The gaitvector is invalid! <break>"
            return False

        if type(liftvector) is not tuple:
            print "The liftvector is invalid! <break>"
            return False

        gaitvector=np.array(gaitvector)
        liftvector=np.array(liftvector)

        halfgaitvector=gaitvector/2 # Rounds to integer...meaningless

        # Stance (5/6 phases):
        nbrstepsstance=phasenbrsteps*5

        # Swing (1/6 phases):
        nbrstepslift=3
        nbrstepsforward=phasenbrsteps-(2*nbrstepslift)

        self.nbrgaitsteps=2*nbrstepslift+nbrstepsforward+nbrstepsstance

        limbcount=0
        for index in seq:

            limb=self.hexmodel.limbs[index]

            pt={} # All end points of the limb movement
            self.limbsgaitpositions[index]=[] # Unsets previous limb's paths 

            # Load limb data
            coxalength=limb.hip.length
            femurlength=limb.femur.length
            tibialength=limb.tibia.length
            origin=limb.origin

            if self.gaiton:
                position = self.limbslastpositions[index]
            else:
                position = limb.getposition()

            halfgaitvector=gaitvector/2
            if limb.side == 'left':
                halfgaitvector[0] = -halfgaitvector[0]
            elif limb.side == 'right':
                pass
            else:
                print "Left/Right limb side unknown! <break>"
                return False


            pt['backdown']=position-halfgaitvector
            pt['backup']=position-halfgaitvector+liftvector
            pt['forwup']=position+halfgaitvector+liftvector
            pt['forwdown']=position+halfgaitvector

            
            # Creation of the cycle
            self.limbsgaitpositions[index]=CyclicList()  # List that will contain all angular solutions for each tick of limb movement
            self.limbsgaitpositions[index].extend(ik.generatelinearpath(coxalength, femurlength, tibialength, 
                                                                        origin, 
                                                                        pt['backdown'], pt['backup'], 
                                                                        nbrstepslift))  # Move up

            self.limbsgaitpositions[index].extend(ik.generatelinearpath(coxalength, femurlength, tibialength, 
                                                                        origin, 
                                                                        pt['backup'], pt['forwup'], 
                                                                        nbrstepsforward))  # Swing forward

            self.limbsgaitpositions[index].extend(ik.generatelinearpath(coxalength, femurlength, tibialength, 
                                                                        origin, 
                                                                        pt['forwup'], pt['forwdown'], 
                                                                        nbrstepslift))  # Move down

            self.limbsgaitpositions[index].extend(ik.generatelinearpath(coxalength, femurlength, tibialength, 
                                                                        origin, 
                                                                        pt['forwdown'], pt['backdown'], 
                                                                        nbrstepsstance))  # Stance back

            # Finally performs the necessary phase shift of the list
            # limbindex 1 will have no shift; index 2 will shift 1x[phasenbrsteps]; index 3 will shift 2x[phasenbrsteps];
            self.limbsgaitpositions[index].setindexzero((limbcount)*phasenbrsteps)
            limbcount+=1 # increments counter


        #self.printpaths()

    def patharraysloaded(self):
        """Checks if the there are any paths loaded for any member"""

        try:
            for limbindex, patharray in self.limbsgaitpositions.items():
                if len(patharray)>0:
                    return True
        except:
            return False            

        return False

    def repositionmembersforgaitstart(self, liftheight):
        """ Lifts a predefined height and repositions each member individually for the gait starting positions """
        print('Limbs last positions:')
        print(self.limbslastpositions)
        for limbindex, patharray in self.limbsgaitpositions.items():
            limb=self.hexmodel.limbs[limbindex] # Gets the reference of the limb
            (jointsangles, endposition)=patharray[0] # Gets the limb's gait starting position  

            p0=limb.getposition() # Limb's current positions
            # Saves the limb's current position:
            self.limbslastpositions[limbindex]=copy(p0)

            p2=endposition        # Limb's end positions

            p1=list(self.midpoint(p0, p2)) # Limb's midpoint between current and end position
           
            p1[2]+=liftheight 
            #pdb.set_trace()
            print('Moving limb %s' % limbindex)
            print('from [x={:> 4.1f} y={:> 4.1f} z={:> 4.1f}] to [x={:> 4.1f} y={:> 4.1f} z={:> 4.1f}]'.format(p0[0],p0[1],p0[2],p2[0],p2[1],p2[2]))
            self.quadraticmove(limb, p1, p2)

    def repositionmembersaftergait(self, liftheight):
        """ Lifts a predefined height and repositions each member individually on position before gait """
        # TODO: First lower the lifted limb!!!
        print('Respositioning limbs after gait stop to last positions')
        # Calculate the lower Z of the limbs (Ground Plane)
        minz = 99999
        for limbindex, patharray in self.limbsgaitpositions.items():
            limbposition=self.hexmodel.limbs[limbindex].getposition()
            if limbposition[2] < minz:
                minz = limbposition[2]

        for limbindex, patharray in self.limbsgaitpositions.items():
            limb=self.hexmodel.limbs[limbindex] # Gets the reference of the limb 
            
            p0=limb.getposition() # Limb's current positions
            p2=self.limbslastpositions[limbindex] # Limb's end positions

            p1=list(self.midpoint(p0, p2))

            if p1[2] < minz+liftheight:
                p1[2] = minz+liftheight # Limb's midpoint raised [liftheight] in Z axis 
            #print('Points for quadratic move p0={:> 4.1f}  p1={:> 4.1f} p2={:> 4.1f}'.format(p0[2], p1[2], p2[2]))
            #pdb.set_trace()
            self.quadraticmove(limb, p1, p2)

    def startgait(self):
        """Starts the loaded gait"""
        if not self.patharraysloaded():
            print 'Cannot initiate gait start: Paths not loaded!'
            return False

        if self.gaiton:
            self.stopgait()
            self.repositionmembersaftergait(30)

        print('Respositioning limbs for gait start ')
        print(self.hexmodel)
        self.repositionmembersforgaitstart(30)
        print('Limbs repositioned. Starting gait. ')
        print(self.hexmodel)
        self.gaiton = True                   

        self.gaitthread = threading.Thread(target=self.updategait, args=([0.01]))
        self.gaitthread.start()

        return True

    def stopgait(self):
        """Stops the loaded gait"""
        self.gaiton = False
        self.gaitthread.join() # Waits for the thread to end
        # TODO: finally: reposiciona as patas na posicao inicial

        return True

    def startmarch(self):
        """Starts looped march test in the same spot"""

        try:
            while True:
                m.moveLimbsTipTo([1, 2, 3, 4, 5, 6], [80, -42])
                m.moveLimbsTipTo([1, 2, 3, 4, 5, 6], [80, -140])
                m.moveLimbsTipTo([1, 2, 3, 4, 5, 6], [80, -42])
                m.moveLimbsTipTo([1, 3, 5], [80, -140])
                m.moveLimbsTipTo([1, 3, 5], [80, -42])
                m.moveLimbsTipTo([2, 4, 6], [80, -140])
                m.moveLimbsTipTo([2, 4, 6], [80, -42])
                m.moveLimbsTipTo([1, 2, 3, 4, 5, 6], [80, -35])
                #time.sleep(0.01)

        except KeyboardInterrupt:
            pass

        return True

    def poweron(self):
        if not self.powered:
            self.standup()
            print('Hexmodel: POWER ON')

    def poweroff(self):
        if self.gaiton:
            self.stopgait()

        if self.powered:
            for limbindex, limb in  self.hexmodel.limbs.items():
                limb.poweroff()
            self.powered = False
            print('Hexmodel: POWER OFF')

    def printpaths(self):
        
        for s in range(0, len(self.limbsgaitpositions[1])):
            row=''
            for limbindex in range(1,7):
                pos = self.limbsgaitpositions[limbindex][s][1]
                row += '|{:> 5.0f},{:> 5.0f},{:> 5.0f}|'.format(pos[0], pos[1], pos[2])
            print(row)


####### END OF FILE #####################################################################
