from __future__ import division
from visual import *
import math
import ik

def calcVectLength(Vector):
    distance=math.sqrt(Vector[0]**2+Vector[1]**2+Vector[2]**2)
    return distance

def midPoint(Point1, Point2):
    mid=((Point2[0]-Point1[0])/2+Point1[0], (Point2[1]-Point1[1])/2+Point1[1], (Point2[2]-Point1[2])/2+Point1[2])
    return mid

def dispVector(Point1, Point2):
    deltaX=Point2[0]-Point1[0]
    deltaY=Point2[1]-Point1[1]
    deltaZ=Point2[2]-Point1[2]
    return vector(deltaX, deltaY, deltaZ)

def calcPtsDistance(Point1, Point2):
    return calcVectLength(dispVector(Point1, Point2))

class hexBone:
    #Represent's each member of hex's limb
    def __init__(self, boneInitialPosition, boneFinalPosition, boneColor=color.red):
        boneheight=1
        bonewidth=0.1
        boneLength=calcPtsDistance(boneInitialPosition, boneFinalPosition)
        self.length=boneLength
        boneAxis=dispVector(boneInitialPosition, boneFinalPosition)
        boneCenter=midPoint(boneInitialPosition, boneFinalPosition)
        self.geometry=box(pos=(boneCenter), axis=boneAxis, height=boneheight, width=bonewidth, color=boneColor)
        print 'Bone drawn at'+str(boneCenter)
        self.joint=sphere(pos=(boneInitialPosition), radius=boneheight/2, color=color.red)
        self.jointaxis=vector(0,0,1) #Assumes 2D
        self.angle=0
        self.childBones=[]

    def rotate(self, degrees, rotationAxis=None, rotationOrigin=None):
        #Recursive function after(including) first child rotation axis and origin of rotation are from parent bone
        if rotationAxis is None:
            rotationAxis=self.jointaxis
        if rotationOrigin is None:
            rotationOrigin=self.joint.pos
        
        #Rotate all attached child bones
        for bone in self.childBones:
            bone.rotate(degrees, rotationAxis, rotationOrigin)
        
        #Finally rotate self
        self.geometry.rotate(angle=radians(degrees), axis=rotationAxis, origin=rotationOrigin)
        #update current absolute angle
        self.angle+=degrees
        self.joint.rotate(angle=radians(degrees), axis=rotationAxis, origin=rotationOrigin)

    def attachBone(self, bone):
        if isinstance(bone, hexBone):
            self.childBones.append(bone)
        else:
            print "Bone attached is not ok"

class hexLimb:
    #Represent's one Hex's limb
    def __init__(self, femurLength, tibiaLength, femurAngle=30, tibiaAngle=-90):
        self.femur=hexBone(vector(0,0,0),vector(femurLength,0,0), color.green)
        self.tibia=hexBone(vector(femurLength,0,0), vector(femurLength+tibiaLength,0,0), color.magenta)
        self.femur.attachBone(self.tibia)

        #Move limb away from singularity situation
        self.femur.rotate(femurAngle)
        self.tibia.rotate(tibiaAngle)

    def tipPosition(self):
        origin=self.femur.joint.pos
        femurLength=self.femur.length
        femurAngle=self.femur.angle
        tibiaLength=self.tibia.length
        tibiaAngle=self.tibia.angle
        
        x=origin.x+femurLength*math.cos(math.radians(femurAngle))+tibiaLength*math.cos(math.radians(tibiaAngle))
        y=origin.y+femurLength*math.sin(math.radians(femurAngle))+tibiaLength*math.sin(math.radians(tibiaAngle))
        z=0

        return vector(x, y, z)
    
    def moveTipTo(self, endPosition):
        i=0
        while calcPtsDistance(self.tipPosition(), endPosition) > 0.01:
            rate(100)
            rotIteration=ik.ik2DOFJacobian(limb.femur.length,
                                           limb.tibia.length,
                                           limb.femur.angle,
                                           limb.femur.angle-limb.tibia.angle,
                                           limb.femur.joint.pos.x,limb.femur.joint.pos.y,
                                           endPosition[0],endPosition[1])
            limb.femur.rotate(rotIteration[0])
            limb.tibia.rotate(-rotIteration[1])
            i+=1 

        print 'Current Distance to Target:' + str(calcPtsDistance(self.tipPosition(), endPosition))

def handlesMouseClick(evt):
    #Returns mouse click coords on the xy plane
    global limb
    loc=evt.project(normal=(0,0,1))
    if loc:
        limb.moveTipTo(vector(loc))
                                         
scene.range=40
limb=hexLimb(20, 10)
limb.moveTipTo(vector(-10,-5,0))
scene.bind('click', handlesMouseClick)
   

