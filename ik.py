import math

def ik2DOFJacobian(Length1, Length2, alpha1, alpha2, oriX, oriY,  x, y):
    maxIterations=300
    L1=Length1
    L2=Length2
    x=float(x)
    y=float(y)
    

    theta1=alpha1/(180/math.pi)
    theta2=alpha2/(180/math.pi)
    
    x0=oriX+L1*math.cos(theta1)+L2*math.cos(theta1-theta2)
    y0=oriY+L1*math.sin(theta1)+L2*math.sin(theta1-theta2)

    iDist=math.sqrt((x-x0)**2+(y-y0)**2)

    #print("Current Distance:"+str(round(iDist,2))+"xRot:"+str(int(xRot))+"yRot:"+str(int(yRot)))
    x1=(x-x0)/1
    y1=(y-y0)/1

    if iDist>0:
        Jacobian = [[0,0],[0,0]]
        Jacobian[0][0]=-L1*math.sin(theta1)-L2*math.sin(theta1-theta2)
        Jacobian[0][1]=L2*math.sin(theta1-theta2)
        Jacobian[1][0]=L1*math.cos(theta1)+L2*math.cos(theta1-theta2)
        Jacobian[1][1]=-L2*math.cos(theta1-theta2)

        det=Jacobian[0][0]*Jacobian[1][1]-Jacobian[0][1]*Jacobian[1][0]
        if (det==0):
            det=1.0
        
        invJacobian = [[0,0],[0,0]]
        invJacobian[0][0]=Jacobian[1][1]/det
        invJacobian[0][1]=-Jacobian[0][1]/det
        invJacobian[1][0]=-Jacobian[1][0]/det
        invJacobian[1][1]=Jacobian[0][0]/det

        deltaAlpha1=invJacobian[0][0]*(x1)+invJacobian[0][1]*(y1)
        deltaAlpha2=invJacobian[1][0]*(x1)+invJacobian[1][1]*(y1)

        alpha1+=deltaAlpha1
        alpha2+=deltaAlpha2
         
    return (deltaAlpha1, deltaAlpha2)
