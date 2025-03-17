from math import pi, sin, cos
import numpy as np
import pygame

import src.config.puntos_torso as torso
import src.config.colores as color
import src.config.coordenadas as coor
from src.utils.cinematica import FK
from src.animation.display import display_rotate

#pygame.init()
screen = pygame.display.set_mode((600, 600)) 

class SpotAnime:
    
    def animate(self,pos,t,thetax,thetaz,angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,walking_speed,walking_direction,steering,stance):  
        
        theta_spot = pos[12]
        x_spot = pos[13]
        y_spot = pos[14]                 
        z_spot = pos[15]
        CGabs = [x_spot[7],y_spot[7],z_spot[7]]
        dCG = [x_spot[8],y_spot[8],z_spot[8]]
        
            
        screen.fill(color.WHITE)
        #pos = torso.walking (track,x_offset,steering_radius,steering_angle,cw,h_amp,v_amp,b_height,ra_longi,ra_lat,0.2,t)
        
        "Floor Display"""
        line = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[theta_spot[0],theta_spot[1],0,0,0,0],thetax,thetaz,coor.xFloor,coor.yFloor,coor.zFloor)
        pygame.draw.polygon(screen,color.GREY,line,0)
        
        """Floor Grid Display """
        for i in range (0,11):
                 line = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[theta_spot[0],theta_spot[1],0,0,0,0],thetax,thetaz,[-500+i*100,-500+i*100],[-500,500],[0,0])
                 pygame.draw.lines(screen,color.DARK_GREY,False,line,1)
                 line = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[theta_spot[0],theta_spot[1],0,0,0,0],thetax,thetaz,[-500,500],[-500+i*100,-500+i*100],[0,0])
                 pygame.draw.lines(screen,color.DARK_GREY,False,line,1)
        
        """ X,Y,Z frame display"""
        line = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,coor.xX,coor.yX,coor.zX)
        pygame.draw.lines(screen,color.RED,False,line,2)
        
        line = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,coor.xY,coor.yY,coor.zY)
        pygame.draw.lines(screen,color.GREEN,False,line,2)
        
        line = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,coor.xZ,coor.yZ,coor.zZ)
        pygame.draw.lines(screen,color.BLUE,False,line,2)
        
        """ Radius display """
        center_display = True
        if (steering<2000):
            lineR = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,[center_x,x_spot[0]],[center_y,y_spot[0]],[0,0])
        else:  
            center_x1 = x_spot[0] + (center_x-x_spot[0])/steering*2000
            center_y1 = y_spot[0] + (center_y-y_spot[0])/steering*2000
            lineR = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,[center_x1,x_spot[0]],[center_y1,y_spot[0]],[0,0])
            center_display = False

        """ Direction display """
        xd = x_spot[0] + walking_speed*cos(theta_spot[2]+ walking_direction-pi/2)
        yd = y_spot[0] + walking_speed*sin(theta_spot[2]+ walking_direction-pi/2)
        lineD = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,[xd,x_spot[0]],[yd,y_spot[0]],[0,0])    
        
        """Legs lines"""  
       
        leglf = FK(thetalf,1)
        legrf = FK(thetarf,-1)
        legrr = FK(thetarr,-1)
        leglr = FK(thetalr,1)
        
        """ Center of Gravity """       
        #Calculation of CG absolute position
        lineCG = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,[CGabs[0],CGabs[0]],[CGabs[1],CGabs[1]],[0,CGabs[2]]) 
        
        xleglf =[torso.xlf,torso.xlf+leglf[0],torso.xlf+leglf[1],torso.xlf+leglf[2],torso.xlf+pos[0]]
        yleglf =[torso.ylf,torso.ylf+leglf[3],torso.ylf+leglf[4],torso.ylf+leglf[5],torso.ylf+pos[1]]
        zleglf =[0,leglf[6],leglf[7],leglf[8],pos[2]]    
        linelf = display_rotate (x_spot[1]-x_spot[0],y_spot[1]-y_spot[0],z_spot[1]-z_spot[0],theta_spot,thetax,thetaz,xleglf,yleglf,zleglf)
        
        xlegrf =[torso.xrf,torso.xrf+legrf[0],torso.xrf+legrf[1],torso.xrf+legrf[2],torso.xrf+pos[3]]
        ylegrf =[torso.yrf,torso.yrf+legrf[3],torso.yrf+legrf[4],torso.yrf+legrf[5],torso.yrf+pos[4]]
        zlegrf =[0,legrf[6],legrf[7],legrf[8],pos[5]]
        linerf = display_rotate (x_spot[1]-x_spot[0],y_spot[1]-y_spot[0],z_spot[1]-z_spot[0],theta_spot,thetax,thetaz,xlegrf,ylegrf,zlegrf)
        
        xlegrr =[torso.xrr,torso.xrr+legrr[0],torso.xrr+legrr[1],torso.xrr+legrr[2],torso.xrr+pos[6]]
        ylegrr =[torso.yrr,torso.yrr+legrr[3],torso.yrr+legrr[4],torso.yrr+legrr[5],torso.yrr+pos[7]]
        zlegrr = [0,legrr[6],legrr[7],legrr[8],pos[8]]
        linerr = display_rotate (x_spot[1]-x_spot[0],y_spot[1]-y_spot[0],z_spot[1]-z_spot[0],theta_spot,thetax,thetaz,xlegrr,ylegrr,zlegrr)    
        
        xleglr =[torso.xlr,torso.xlr+leglr[0],torso.xlr+leglr[1],torso.xlr+leglr[2],torso.xlr+pos[9]]
        yleglr =[torso.ylr,torso.ylr+leglr[3],torso.ylr+leglr[4],torso.ylr+leglr[5],torso.ylr+pos[10]]
        zleglr = [0,leglr[6],leglr[7],leglr[8],pos[11]]
        linelr = display_rotate (x_spot[1]-x_spot[0],y_spot[1]-y_spot[0],z_spot[1]-z_spot[0],theta_spot,thetax,thetaz,xleglr,yleglr,zleglr)
        """ Body frame lines """
        lineb = display_rotate (x_spot[1]-x_spot[0],y_spot[1]-y_spot[0],z_spot[1]-z_spot[0],theta_spot,thetax,thetaz,[torso.xlf,torso.xrf,torso.xrr,torso.xlr,torso.xlf],[torso.ylf,torso.yrf,torso.yrr,torso.ylr,torso.ylf],[torso.zlf,torso.zrf,torso.zrr,torso.zlr,torso.zlf])
        
       
        """Sustentation area lines"""
        #stance = False when leg is lifted from the floor
        
        linesus =[]
        if (stance[0]==True):
            linesus.append(linelf[4])
        if (stance[1]==True):
            linesus.append(linerf[4]) 
        if (stance[2]==True):
            linesus.append(linerr[4])
        if (stance[3]==True):
            linesus.append(linelr[4])  
            
        """ Center of gravity into sustentation area """
        linedCG = display_rotate (-x_spot[0],-y_spot[0],-z_spot[0],[0,0,0,0,0,0],thetax,thetaz,[CGabs[0],dCG[0]],[CGabs[1],dCG[1]],[0,0]) 
    
        pygame.draw.polygon(screen,color.VIOLET,linesus,0)    
        pygame.draw.lines(screen,color.BLACK,True,linesus,1)  
        pygame.draw.lines(screen,color.CYAN,False,lineR,2) 
        pygame.draw.lines(screen,color.GREEN,False,lineD,2) 
        
        if (center_display == True):
            pygame.draw.circle(screen,color.BLACK,lineR[0],5)
        pygame.draw.lines(screen,color.BLACK,False, linedCG,1)    
        pygame.draw.circle(screen,color.DARK_CYAN,lineR[1],5)
        
        pygame.draw.lines(screen,color.BLACK,False, lineCG,1)
        if (dCG[2] == True):
            pygame.draw.circle(screen,color.GREEN,lineCG[0],3)
        else:
            pygame.draw.circle(screen,color.RED,lineCG[0],3)
            
        pygame.draw.circle(screen,color.DARK_GREY,lineCG[1],10)
        pygame.draw.lines(screen,color.RED,False, linelf,4)
        pygame.draw.lines(screen,color.RED,False, linerf,4)
        pygame.draw.lines(screen,color.RED,False, linerr,4)
        pygame.draw.lines(screen,color.RED,False, linelr,4)        
        pygame.draw.lines(screen,color.BLUE,False,lineb,10)
        pygame.draw.lines(screen,color.BLACK, False,[[angle[0]/pi*180/45*300+300,0],[angle[0]/pi*180/45*300+300,50]],5)
        pygame.draw.lines(screen,color.BLACK, False,[[0,angle[1]/pi*180/45*300+300],[50,angle[1]/pi*180/45*300+300]],5)
        
        return
