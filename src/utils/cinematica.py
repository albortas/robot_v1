
import sys, os
sys.path.append(os.getcwd())

from math import pi, sin, asin, cos, acos, sqrt
from src.config.dimensiones import d, L0, L1

def IK (L0, L1, L2, d, x, y, z, side): #Inverse Kinematics
    """
        s = 1 for left leg
        s = -1 for right leg
    """
    t2 = y**2
    t3 = z**2
    t4 = t2+t3
    t5 = 1/sqrt(t4)
    t6 = L0**2
    t7 = t2+t3-t6
    t8 = sqrt(t7)
    t9 = d-t8
    t10 = x**2
    t11 = t9**2
    t15 = L1**2
    t16 = L2**2
    t12 = t10+t11-t15-t16
    t13 = t10+t11
    t14 = 1/sqrt(t13)
    error = False
    try:
        theta1 = side*(-pi/2+asin(t5*t8))+asin(t5*y)
        theta2= -asin(t14*x)+asin(L2*t14*sqrt(1/t15*1/t16*t12**2*(-1/4)+1))
        theta3 =-pi + acos(-t12/2/(L1*L2))
        
    except ValueError:
        print ('ValueError IK')
        error = True  
        theta1=90
        theta2=90
        theta3=90
    
    theta = [theta1, theta2, theta3]
    return (theta,error)

def FK (theta, side): #Forward Kinematics
    """ Calculation of articulation points """
    """
    s = 1 for left leg
    s = -1 for right leg
    """
    x_shoulder1 = 0
    y_shoulder1 = d*sin(theta[0])
    z_shoulder1 = d*cos(theta[0])
    
    x_shoulder2 = 0
    y_shoulder2 =side*L0*cos(theta[0])+d*sin(theta[0])
    z_shoulder2 =side*L0*sin(theta[0])-d*cos(theta[0]) 
            
    x_elbow = -L1*sin(theta[1])
    y_elbow = side*L0*cos(theta[0])-(-d-L1*cos(theta[1]))*sin(theta[0])
    z_elbow = side*L0*sin(theta[0]) +(-d-L1*cos(theta[1]))*cos(theta[0])
    
    return [x_shoulder1,x_shoulder2,x_elbow,y_shoulder1,y_shoulder2,y_elbow,z_shoulder1,z_shoulder2,z_elbow]