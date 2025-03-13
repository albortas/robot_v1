import numpy as np
from math import cos, sin
from src.config.puntos_torso import *
from src.config.parametros_caminar import b_height, seq, stepl
from src.utils.operador import xyz_rotation_matrix, new_coordinates
from .ControlFase import ControlFase
class ActualizarPosicion():
    def __init__(self, x_offset, steering, walking_direction, cw, h_amp,t, tstep, theta_spot, x_spot, y_spot, z_spot, step_phase):
        self.control = ControlFase(steering, walking_direction, step_phase, tstep, cw, h_amp, t, x_spot, y_spot)
        self.theta_spot_updated = theta_spot
        self.tstep = tstep
        self.x_offset = x_offset
        self.cw = cw
        self.x_spot = x_spot
        self.y_spot = y_spot
        self.z_spot = z_spot
        self.CG = [x_spot[6],y_spot[6],z_spot[6]]
        self.t1 = t % 1
            
    def posicion_abs_pie(self):
        """ Foot nominal center absolute position"""
        xc, yc = self.control.area_mov.coordenas_circulo()
        Ms = xyz_rotation_matrix (0,0,self.theta_spot_updated[2],False)
        s = new_coordinates(Ms,xc,yc,0,self.x_spot[0], self.y_spot[0],self.z_spot[0])
        dtheta = self.control.delta_theta()
        dMs = xyz_rotation_matrix (0,0,dtheta,False)
        foot_center = new_coordinates(dMs,self.x_spot[0]-s[0], self.y_spot[0]-s[1],0,s[0],s[1],0)
        return foot_center
    
    def actualizar_posicion(self):
        
        dtheta = self.control.delta_theta()
        self.theta_spot_updated[2] += dtheta 
        #Matriz del marco corporal local al marco espacial absoluto
        Ms_updated = xyz_rotation_matrix (self.theta_spot_updated[3],self.theta_spot_updated[4],self.theta_spot_updated[2]+self.theta_spot_updated[5],False)
        #Matriz del marco espacial absoluto al marco corporal local
        Msi_updated = xyz_rotation_matrix (-self.theta_spot_updated[3],-self.theta_spot_updated[4],-(self.theta_spot_updated[2]+self.theta_spot_updated[5]),True)
        
        foot_center = self.posicion_abs_pie()
        
        
        v_amp_t, kcomp = self.control.compensacion_theta()
        
        alpha, alphav = self.control.control_fase()
        
        x_abs_comp, y_abs_comp = self.control.compensacion()
        
        Msi_comp = xyz_rotation_matrix (0,0,-self.theta_spot_updated[2],True)
        #compensation calculation in body center frame
        comp= new_coordinates(Msi_comp,x_abs_comp-self.x_spot[0],y_abs_comp-self.y_spot[0],0,0,0,0)
        
        Ms_comp  =  xyz_rotation_matrix (0,0,self.theta_spot_updated[2],False)  
        #Compensation calculation absoltute space frame
        compt =  new_coordinates(Ms_comp,(comp[0]-self.CG[0])*kcomp+ self.x_offset,(comp[1]-self.CG[1])*kcomp,0,0,0,0)
        """ Frame center new position with gravity center compensation, offset and height """
        x_framecenter_comp = foot_center[0] + compt[0]
        y_framecenter_comp = foot_center[1] + compt[1] 
        z_framecenter_comp = b_height
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
            stance = self.control.control_estado()            
            if stance[i] == False:
                #relative position calculation (used for inverse kinematics)
                mangle = self.control.velocidad_angular()
                xc, yc = self.control.area_mov.coordenas_circulo()
                radii, an = self.control.area_mov.radio_angulo()
                
                alphah = an[i]+mangle*alpha[i]*self.cw
                xleg_target = xc + radii[i]*cos(alphah) -(comp[0]-self.CG[0])*kcomp -self.x_offset -x_frame[i]
                yleg_target = yc + radii[i]*sin(alphah) -(comp[1]-self.CG[1])*kcomp -y_frame[i]
                
                leg_current = new_coordinates(Msi_comp,self.x_spot[i+2]-x_framecorner[i],self.y_spot[i+2]-y_framecorner[i],-z_framecorner[i],0,0,0)
                #interpolate between current position and targe
                if ((seq[i]+stepl-self.t1)>self.tstep):
                    xint[i] = leg_current[0]+(xleg_target - leg_current[0])*(self.tstep)/(seq[i]+stepl-self.t1)
                    yint[i] = leg_current[1]+(yleg_target - leg_current[1])*(self.tstep)/(seq[i]+stepl-self.t1)
                else:
                    xint[i] = xleg_target 
                    yint[i] = yleg_target   
                zint[i] = leg_current[2] + v_amp_t*(1+sin(alphav[i]))/2                 
                #print (leg_current[2],zint[i],leg_current[2]-zint[i])
                Msi_body = xyz_rotation_matrix (-self.theta_spot_updated[3],-self.theta_spot_updated[4],-self.theta_spot_updated[5],True) 
                legs = new_coordinates(Msi_body,xint[i],yint[i],zint[i],0,0,0)
                xleg[i]= legs[0]
                yleg[i]= legs[1]
                zleg[i]= legs[2]
                
                #absolute foot position 
                #Msb_updated = Spot.xyz_rotation_matrix (0,0,theta_spot_updated[2]+theta_spot_updated[5],False)
                foot_abs = new_coordinates(Ms_updated,xleg[i],yleg[i],zleg[i],x_framecorner[i],y_framecorner[i],z_framecorner[i])
                

                xabs[i] = foot_abs[0]
                yabs[i] = foot_abs[1]
                zabs[i] = foot_abs[2]
                        
            else:
                xabs[i] = self.x_spot[i+2]
                yabs[i] = self.y_spot[i+2]
                zabs[i] = 0
                
                #relative foot position of foot on the ground/floor for inverse kinematics
                leg = new_coordinates(Msi_updated,xabs[i]-x_framecorner[i],yabs[i]-y_framecorner[i],zabs[i]-z_framecorner[i],0,0,0)
                xleg[i] = leg[0]
                yleg[i] = leg[1]
                zleg[i] = leg[2]
                
        x_spot_updated =  [foot_center[0],x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3],self.x_spot[6],self.x_spot[7],self.x_spot[8]] 
        y_spot_updated =  [foot_center[1],y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3],self.y_spot[6],self.y_spot[7],self.y_spot[8]] 
        z_spot_updated =  [foot_center[2],z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3],self.z_spot[6],self.z_spot[7],self.z_spot[8]]

        pos = [xleg[0],yleg[0],zleg[0],xleg[1],yleg[1],zleg[1],xleg[2],yleg[2],zleg[2],xleg[3],yleg[3],zleg[3],self.theta_spot_updated,x_spot_updated,y_spot_updated,z_spot_updated]    
        return pos