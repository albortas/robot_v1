from math import pi
from src.config.parametros_caminar import stepl, seq, v_amp
from .AreaMovimientoPie import AreaMovimientoPie


class ControlFase:
    def __init__(self, dir_radio, dir_angulo, step_phase, tstep, cw, h_amp,t,x_spot, y_spot):
        self.area_mov = AreaMovimientoPie(dir_radio,dir_angulo)
        self.x_spot = x_spot
        self.y_spot = y_spot         
        self.step_phase = step_phase
        self.tstep = tstep
        self.cw = cw
        self.h_amp = h_amp
        self.t1 = t % 1
         
    def velocidad_angular(self):
        """ Motion angle """
        radii, an = self.area_mov.radio_angulo()
        maxr = max(radii)
        mangle = self.h_amp/maxr
        return mangle
            
    def delta_theta(self):
        mangle = self.velocidad_angular()
        if (self.step_phase =='start')|(self.step_phase == 'stop'):           
            dtheta = mangle/(1-stepl)*self.tstep/2*self.cw
        else:
            dtheta = mangle/(1-stepl)*self.tstep*self.cw
        return dtheta
    
    def control_fase(self):
        alpha = [0] *4
        alphav = [0] *4
        
        for i in range (0,4):
            alphav[i] =0 
            if (self.t1<(seq[i]+stepl)):                    
                alphav[i] = -pi/2 + 2*pi/stepl * (self.t1 - seq[i])
                t2 = seq[i]+stepl
                if (self.step_phase == 'start'):
                    #End position alpha 
                    alpha[i] = -seq[i]/(1 - stepl)/2 + (t2 - seq[i])/stepl/(1-stepl) * seq[i]  
                if (self.step_phase == 'stop'):                          
                    alpha[i] = -1/2 + seq[i]/(1 - stepl) / 2 + (t2 - seq[i])/stepl * (1 - seq[i]/(1 - stepl)) 
                if (self.step_phase == 'walk'):                                                 
                    alpha[i] = -1/2  + ((t2 - seq[i])/stepl) 
            
        return alpha, alphav
    
    def control_estado(self):
        stance = [True,True,True,True]
        for i in range(4):
            if (self.t1<= seq[i]):
                stance[i] = True #Leg is on the ground (absolute position value unchanged)
            else:
                if (self.t1<(seq[i]+stepl)):                    
                    stance[i] = False #leg is lifted (absolute position value changes)
                else:         
                    stance[i] = True #Leg is on the ground (absolute position value unchanged)
        return stance
    
    def compensacion(self):
        stance = self.control_estado()
        stance_test = sum(stance) #if sum = 4 all feet are on the floor --> body balance
        
        #absolute stance area target point
        #Barycenter of sustentation area with higher weight of diagonal points
        weight = 1.2
        x_abs_area = [0]*4
        y_abs_area = [0]*4
        
        secuencia = [
                    ([3, 5], 4),
                    ([2, 4], 5),
                    ([3, 5], 2),
                    ([2, 4], 3)
        ]
        
        for i, (idx, t_idx) in enumerate(secuencia):
            x_abs_area[i] = ((self.x_spot[idx[0]]+self.x_spot[idx[1]])*weight+self.x_spot[t_idx])/(2*weight+1) 
            y_abs_area[i] = ((self.y_spot[idx[0]]+self.y_spot[idx[1]])*weight+self.y_spot[t_idx])/(2*weight+1)                                 

        if  (stance_test == 4): 
            istart = 0
            iend = 0
            #identify transition start and target
            tstart = (int(self.t1/0.25)*0.25)
            tend = tstart+0.25
            if (tend==1):
                tend = 0
            
            for i in range (0,4):
                if (tstart == seq[i]):
                    istart = i
                if (tend  == seq[i]):
                    iend = i
            
            if (self.t1>(seq[istart]+stepl)):        
                x_abs_comp= x_abs_area[istart]+(x_abs_area[iend]-x_abs_area[istart])*(self.t1-tstart-stepl)/(0.25-stepl)
                y_abs_comp= y_abs_area[istart]+(y_abs_area[iend]-y_abs_area[istart])*(self.t1-tstart-stepl)/(0.25-stepl) 
            else:
                x_abs_comp = x_abs_area[istart]
                y_abs_comp = y_abs_area[istart] 
        else:
            for i in range (0,4):
                if (stance[i]==0):
                    x_abs_comp = x_abs_area[i] 
                    y_abs_comp = y_abs_area[i]
                    
        return x_abs_comp, y_abs_comp

    def compensacion_theta(self):
        kcomp = 1
        v_amp_t = v_amp
        ts = 0.25
        if (self.step_phase == 'start'):
            if (self.t1< ts):
                kcomp = self.t1/ts
                v_amp_t = 0
        elif (self.step_phase == 'stop'):  
            if (self.t1 > (1-ts)):
                kcomp = (1- self.t1)/ts
                v_amp_t = 0
        return v_amp_t, kcomp
                
    
            
        
        