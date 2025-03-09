import sys,os
sys.path.append(os.getcwd())

from math import sqrt
from src.utils.cinematica_cg import FK_Weight
from src.config.inercia import Weight_Body, Weight_Shoulder, Weight_Leg, Weight_Foreleg, xCG_Body, yCG_Body, zCG_Body
import src.config.puntos_torso as torso

class SpotCG:
    def CG_calculation (self,thetalf,thetarf,thetarr,thetalr):
        cgposlf=(FK_Weight(thetalf,1))
        cgposrf=(FK_Weight(thetarf,-1))
        cgposrr=(FK_Weight(thetarr,-1))
        cgposlr=(FK_Weight(thetalr,1))
        
        Weightsum = Weight_Body+4*(Weight_Shoulder+Weight_Leg+Weight_Foreleg)
        
        xcgrf=(cgposrf[0]+torso.xrf)*Weight_Shoulder+(cgposrf[1]+torso.xrf)*Weight_Leg+(cgposrf[2]+torso.xrf)*Weight_Foreleg
        xcglf=(cgposlf[0]+torso.xlf)*Weight_Shoulder+(cgposlf[1]+torso.xlf)*Weight_Leg+(cgposlf[2]+torso.xlf)*Weight_Foreleg
        xcgrr=(cgposrr[0]+torso.xrr)*Weight_Shoulder+(cgposrr[1]+torso.xrr)*Weight_Leg+(cgposrr[2]+torso.xrr)*Weight_Foreleg
        xcglr=(cgposlr[0]+torso.xlr)*Weight_Shoulder+(cgposlr[1]+torso.xlr)*Weight_Leg+(cgposlr[2]+torso.xlr)*Weight_Foreleg
        xcg= (xcglf+xcgrf+xcgrr+xcglr+xCG_Body*Weight_Body)/Weightsum
        
        ycglf=(cgposlf[3]+torso.ylf)*Weight_Shoulder+(cgposlf[4]+torso.ylf)*Weight_Leg+(cgposlf[5]+torso.ylf)*Weight_Foreleg
        ycgrf=(cgposrf[3]+torso.yrf)*Weight_Shoulder+(cgposrf[4]+torso.yrf)*Weight_Leg+(cgposrf[5]+torso.yrf)*Weight_Foreleg
        ycgrr=(cgposrr[3]+torso.yrr)*Weight_Shoulder+(cgposrr[4]+torso.yrr)*Weight_Leg+(cgposrr[5]+torso.yrr)*Weight_Foreleg
        ycglr=(cgposlr[3]+torso.ylr)*Weight_Shoulder+(cgposlr[4]+torso.ylr)*Weight_Leg+(cgposlr[5]+torso.ylr)*Weight_Foreleg
        ycg= (ycglf+ycgrf+ycgrr+ycglr+yCG_Body*Weight_Body)/Weightsum
        
        zcglf=(cgposlf[6]+torso.zlf)*Weight_Shoulder+(cgposlf[7]+torso.zlf)*Weight_Leg+(cgposlf[8]+torso.zlf)*Weight_Foreleg
        zcgrf=(cgposrf[6]+torso.zrf)*Weight_Shoulder+(cgposrf[7]+torso.zrf)*Weight_Leg+(cgposrf[8]+torso.zrf)*Weight_Foreleg
        zcgrr=(cgposrr[6]+torso.zrr)*Weight_Shoulder+(cgposrr[7]+torso.zrr)*Weight_Leg+(cgposrr[8]+torso.zrr)*Weight_Foreleg
        zcglr=(cgposlr[6]+torso.zlr)*Weight_Shoulder+(cgposlr[7]+torso.zlr)*Weight_Leg+(cgposlr[8]+torso.zlr)*Weight_Foreleg
        zcg= (zcglf+zcgrf+zcgrr+zcglr+zCG_Body*Weight_Body)/Weightsum
        
        return (xcg,ycg,zcg)
    
    
    def CG_distance (self,x_legs,y_legs,z_legs,xcg,ycg,stance):
        
        #line equation c * x + s * y - p  = 0
        # with c = a/m et s = b/m
         
        a1 = (y_legs[0]-y_legs[2])
        b1 = -(x_legs[0]-x_legs[2])
        m1 =sqrt(a1**2 + b1**2)
        c1 = a1/m1
        s1 = b1/m1
        
        a2 = (y_legs[1]-y_legs[3])
        b2 = -(x_legs[1]-x_legs[3])
        m2 =sqrt(a2**2 + b2**2)
        c2 = a2/m2
        s2 = b2/m2   
        
        p1 = c1*x_legs[0] + s1*y_legs[0]
        p2 = c2*x_legs[1] + s2*y_legs[1]
        
        """ Dstance calculation """
        d1 = c1*xcg + s1*ycg - p1
        d2 = c2*xcg + s2*ycg - p2
        
        """ intersection calculation """
        #perpendicalar line equation -s * x + c * y - q = 0
        
        q1 = -s1*xcg +c1*ycg
        q2 = -s2*xcg +c2*ycg
        
        xint1 = c1*p1 - s1*q1
        yint1 = c1*q1 + s1*p1
        
        xint2 = c2*p2 - s2*q2
        yint2 = c2*q2 + s2*p2
        
        """ Check if inside sustentation triangle """
        d = 0
        xint = xcg
        yint = ycg
        if (stance[0]== False)|(stance[2]== False): 
            d = d2
            xint = xint2
            yint = yint2
            
        
        if (stance[1]== False)|(stance[3]== False): 
            d = d1
            xint = xint1
            yint = yint1  
            
        balance = True
    
        if (stance[0] == False)&(d< 0):
            balance = False
            
        if (stance[1] == False)&(d> 0):
            balance = False    
        
        if (stance[2] == False)&(d> 0):
            balance = False    
            
        if (stance[3] == False)&(d< 0):
            balance = False  
            
        if (balance == False):
            d=-abs(d)
        else:
            d=abs(d)
        
        return (d,xint,yint,balance)
