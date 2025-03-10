import numpy as np
from math import pi, sin, cos, atan2, sqrt

from src.utils.operador import xyz_rotation_matrix,new_coordinates
from src.config.puntos_torso import *
from src.config.mover_pie import seq

phase = pi/8 # optimum position when leg is fully lifted   
                    
"""
Walking Function that generates the walking positions
"""

def start_walk_stop (track,x_offset,steering_radius, steering_angle,cw,h_amp,v_amp,height,stepl,t,tstep,theta_spot,x_spot,y_spot,z_spot,step_phase):            

    alpha = np.zeros(4)
    alphav = np.zeros(4)

    theta_spot_updated = theta_spot

    CG = [x_spot[6],y_spot[6],z_spot[6]]

    """ Steering center coordinates in spot frame """
    xc = steering_radius* cos(steering_angle)
    yc = steering_radius* sin(steering_angle)
    
    #rotation matrix for frame position
    #Mf = xyz_rotation_matrix (self,frame_pos[0],frame_pos[1],frame_pos[2],False)
    
    Ms = xyz_rotation_matrix (0,0,theta_spot_updated[2],False)

    s = new_coordinates(Ms,xc,yc,0,x_spot[0],y_spot[0],z_spot[0])
    xs = s[0]
    ys = s[1]
    
    """ Nominal Foot Position """        
    xn = [xlf, xrf,xrr, xlr]
    yn = [ylf+track,yrf-track,yrr-track,ylr+track]
    
    radii = np.zeros(4)
    an = np.zeros(4)
    for i in range (0,4): 
        """ Steering radius """ 
        radii[i] = sqrt((xc-xn[i])**2+(yc-yn[i])**2)
        """ Foot nominal angle"""  
        an[i] = atan2(yn[i]-yc,xn[i]-xc)   
    
    """ Motion angle """
    maxr = max(radii)
    mangle = h_amp/maxr 
    
    """ Rotation angle and translation calculation"""    
    if (step_phase =='start')|(step_phase == 'stop'):           
        dtheta = mangle/(1-stepl)*tstep/2*cw
    else:
        dtheta = mangle/(1-stepl)*tstep*cw
    theta_spot_updated[2] = dtheta + theta_spot[2]
    
    #Matrix from local body frame to absolute space frame
    Ms_updated = xyz_rotation_matrix (theta_spot_updated[3],theta_spot_updated[4],theta_spot_updated[2]+theta_spot_updated[5],False)
    #Matrix from absolute space frame to local body frame
    Msi_updated = xyz_rotation_matrix (-theta_spot_updated[3],-theta_spot_updated[4],-(theta_spot_updated[2]+theta_spot_updated[5]),True)
    #Delta rotation matric from from body center to absolute space frame
    dMs = xyz_rotation_matrix (0,0,dtheta,False)
            
    """ Foot nominal center absolute position"""
    foot_center = new_coordinates(dMs,x_spot[0]-xs, y_spot[0]-ys,0,xs,ys,0)         
                    
    t1 = t%1
    kcomp = 1
    stance = [True,True,True,True]

    for i in range (0,4):
        alphav[i] =0 
        if (t1<=seq[i]):
                stance[i] = True #Leg is on the ground (absolute position value unchanged)
        else:        
                if (t1<(seq[i]+stepl)):
                    
                    stance[i] = False #leg is lifted (absolute position value changes)
                    alphav[i] = -pi/2+2*pi/stepl*(t1-seq[i])
                    t2 = seq[i]+stepl
                    if (step_phase == 'start'):
                        #End position alpha 
                        alpha[i] = -seq[i]/(1-stepl)/2 + (t2-seq[i])/stepl/(1-stepl)*seq[i]  
                    if (step_phase == 'stop'):                          
                        alpha[i] = -1/2 + seq[i]/(1-stepl)/2 + (t2-seq[i])/stepl*(1-seq[i]/(1-stepl)) 
                    if (step_phase == 'walk'):                                                 
                        alpha[i] = -1/2  + ((t2-seq[i])/stepl) 
                else:         
                    stance[i] = True #Leg is on the ground (absolute position value unchanged)

    """ Compensation Calculation """
    stance_test = np.sum(stance) #if sum = 4 all feet are on the floor --> body balance
    
    #absolute stance area target point
    #Barycenter of sustentation area with higher weight of diagonal points
    weight = 1.2
    x_abs_area = np.zeros(4)
    y_abs_area = np.zeros(4)
    
    
    x_abs_area[0] = ((x_spot[3]+x_spot[5])*weight+x_spot[4])/(2*weight+1) 
    y_abs_area[0] = ((y_spot[3]+y_spot[5])*weight+y_spot[4])/(2*weight+1)                                 
    x_abs_area[1] = ((x_spot[2]+x_spot[4])*weight+x_spot[5])/(2*weight+1) 
    y_abs_area[1] = ((y_spot[2]+y_spot[4])*weight+y_spot[5])/(2*weight+1)                   
    x_abs_area[2] = ((x_spot[3]+x_spot[5])*weight+x_spot[2])/(2*weight+1) 
    y_abs_area[2] = ((y_spot[3]+y_spot[5])*weight+y_spot[2])/(2*weight+1)  
    x_abs_area[3] = ((x_spot[2]+x_spot[4])*weight+x_spot[3])/(2*weight+1) 
    y_abs_area[3] = ((y_spot[2]+y_spot[4])*weight+y_spot[3])/(2*weight+1) 

    if  (stance_test == 4): 
        istart = 0
        iend = 0
        #identify transition start and target
        tstart = (int(t1/0.25)*0.25)
        tend = tstart+0.25
        if (tend==1):
            tend = 0
        
        for i in range (0,4):
            if (tstart == seq[i]):
                istart = i
            if (tend  == seq[i]):
                iend = i
        
        if (t1>(seq[istart]+stepl)):        
            x_abs_comp= x_abs_area[istart]+(x_abs_area[iend]-x_abs_area[istart])*(t1-tstart-stepl)/(0.25-stepl)
            y_abs_comp= y_abs_area[istart]+(y_abs_area[iend]-y_abs_area[istart])*(t1-tstart-stepl)/(0.25-stepl) 
        else:
            x_abs_comp = x_abs_area[istart]
            y_abs_comp = y_abs_area[istart] 
    else:
        for i in range (0,4):
            if (stance[i]==0):
                x_abs_comp = x_abs_area[i] 
                y_abs_comp = y_abs_area[i]
        
    Msi_comp = xyz_rotation_matrix (0,0,-theta_spot_updated[2],True) 
    #compensation calculation in body center frame
    comp= new_coordinates(Msi_comp,x_abs_comp-x_spot[0],y_abs_comp-y_spot[0],0,0,0,0)                     
    """ Compensation calculation with theta """
    v_amp_t = v_amp
    ts = 0.25
    if (step_phase == 'start'):
        if (t1< ts):
            kcomp = t1/ts
            v_amp_t = 0
    elif (step_phase == 'stop'):  
        if (t1 > (1-ts)):
            kcomp = (1-t1)/ts
            v_amp_t = 0
    Ms_comp  =  xyz_rotation_matrix (0,0,theta_spot_updated[2],False)  
    #Compensation calculation absoltute space frame
    compt =  new_coordinates(Ms_comp,(comp[0]-CG[0])*kcomp+x_offset,(comp[1]-CG[1])*kcomp,0,0,0,0)
    """ Frame center new position with gravity center compensation, offset and height """
    x_framecenter_comp = foot_center[0] + compt[0]
    y_framecenter_comp = foot_center[1] + compt[1] 
    z_framecenter_comp = height
            
    """ New Frame corners position absolute including compensation """
    x_frame = [xlf, xrf, xrr, xlr]
    y_frame = [ylf, yrf, yrr, ylr]
    z_frame = [0,0,0,0]
    
    x_framecorner = np.zeros(4)
    y_framecorner = np.zeros(4)
    z_framecorner = np.zeros(4)

    for i in range (0,4): 
        #Body corners calculation in absolute space frame
        frame_corner = new_coordinates(Ms_updated,x_frame[i],y_frame[i],z_frame[i],x_framecenter_comp,y_framecenter_comp,z_framecenter_comp)
        x_framecorner[i] = frame_corner[0]
        y_framecorner[i] = frame_corner[1]
        z_framecorner[i] = frame_corner[2]


    xleg = np.zeros(4)
    yleg = np.zeros(4)
    zleg = np.zeros(4)
    xabs = np.zeros(4)
    yabs = np.zeros(4)
    zabs = np.zeros(4)
    xint = np.zeros(4)
    yint = np.zeros(4)
    zint = np.zeros(4)
    
    for i in range (0,4):              
        if stance[i] == False:
            #relative position calculation (used for inverse kinematics)
            alphah = an[i]+mangle*alpha[i]*cw
            xleg_target = xc + radii[i]*cos(alphah) -(comp[0]-CG[0])*kcomp -x_offset -x_frame[i]
            yleg_target = yc + radii[i]*sin(alphah) -(comp[1]-CG[1])*kcomp -y_frame[i]
            
            leg_current = new_coordinates(Msi_comp,x_spot[i+2]-x_framecorner[i],y_spot[i+2]-y_framecorner[i],-z_framecorner[i],0,0,0)
            #interpolate between current position and targe
            if ((seq[i]+stepl-t1)>tstep):
                xint[i] = leg_current[0]+(xleg_target - leg_current[0])*(tstep)/(seq[i]+stepl-t1)
                yint[i] = leg_current[1]+(yleg_target - leg_current[1])*(tstep)/(seq[i]+stepl-t1)
            else:
                xint[i] = xleg_target 
                yint[i] = yleg_target   
            zint[i] = leg_current[2] + v_amp_t*(1+sin(alphav[i]))/2                 
            #print (leg_current[2],zint[i],leg_current[2]-zint[i])
            Msi_body = xyz_rotation_matrix (-theta_spot_updated[3],-theta_spot_updated[4],-theta_spot_updated[5],True) 
            legs = new_coordinates(Msi_body,xint[i],yint[i],zint[i],0,0,0)
            xleg[i]= legs[0]
            yleg[i]= legs[1]
            zleg[i]= legs[2]
            
            #absolute foot position 
            #Msb_updated = xyz_rotation_matrix (self,0,0,theta_spot_updated[2]+theta_spot_updated[5],False)
            foot_abs = new_coordinates(Ms_updated,xleg[i],yleg[i],zleg[i],x_framecorner[i],y_framecorner[i],z_framecorner[i])
            

            xabs[i] = foot_abs[0]
            yabs[i] = foot_abs[1]
            zabs[i] = foot_abs[2]
                    
        else:
            xabs[i] = x_spot[i+2]
            yabs[i] = y_spot[i+2]
            zabs[i] = 0
            
            #relative foot position of foot on the ground/floor for inverse kinematics
            leg = new_coordinates(Msi_updated,xabs[i]-x_framecorner[i],yabs[i]-y_framecorner[i],zabs[i]-z_framecorner[i],0,0,0)
            xleg[i] = leg[0]
            yleg[i] = leg[1]
            zleg[i] = leg[2]
            
    x_spot_updated =  [foot_center[0],x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3],x_spot[6],x_spot[7],x_spot[8]] 
    y_spot_updated =  [foot_center[1],y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3],y_spot[6],y_spot[7],y_spot[8]] 
    z_spot_updated =  [foot_center[2],z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3],z_spot[6],z_spot[7],z_spot[8]] 
    
    
    pos = [xleg[0],yleg[0],zleg[0],xleg[1],yleg[1],zleg[1],xleg[2],yleg[2],zleg[2],xleg[3],yleg[3],zleg[3],theta_spot_updated,x_spot_updated,y_spot_updated,z_spot_updated]    
    return pos


    
    
    
        
        
    
