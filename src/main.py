from time import sleep, time
from math import pi, sin, cos, atan, atan2, sqrt
import numpy as np

import pygame
pygame.init()
screen = pygame.display.set_mode((600, 600)) 
pygame.display.set_caption("SPOTMICRO")

from config.puntos_torso import *
from config.dimensiones import L1, L2, Lb, d
from config.parametros_caminar import *
from utils.operador import xyz_rotation_matrix, new_coordinates
from control.control import caminar
from motion.move import moving
from utils.cinematica import IK, FK
from src.motion.ActualizarPosicion import ActualizarPosicion

from centro_gravedad.centro_g import SpotCG
SpotCG = SpotCG()
from animation.animation import SpotAnime
SpotAnim = SpotAnime()


""" Joystick Init """

pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
clock = pygame.time.Clock()

x_spot = [0, x_offset, xlf, xrf, xrr, xlr,0,0,0]
y_spot = [0,0, ylf+track, yrf-track, yrr-track, ylr+track,0,0,0]
z_spot = [0,b_height,0,0,0,0,0,0,0]

"""theta_spot = [x angle ground, y angle ground, z angle body in space, x angle body, y angle body, z angle body] """

#theta xyz of ground then theta xyz of frame/body
pos_init = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height]

thetarf = IK(pos_init[3], pos_init[4], pos_init[5], -1)[0]
thetalf = IK(pos_init[0], pos_init[1], pos_init[2], 1)[0]
thetarr = IK(pos_init[6], pos_init[7], pos_init[8], -1)[0]
thetalr = IK(pos_init[9], pos_init[10], pos_init[11], 1)[0]

CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
#Calculation of CG absolute position
M = xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
CGabs = new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)

x_spot = [0, x_offset, xlf, xrf, xrr, xlr,CG[0],CGabs[0],dCG[1]]
y_spot = [0,0, ylf+track, yrf-track, yrr-track, ylr+track,CG[1],CGabs[1],dCG[2]]
z_spot = [0,b_height,0,0,0,0,CG[2],CGabs[2],dCG[3]]

pos = [-x_offset,track,-b_height,-x_offset,-track,-b_height,-x_offset,-track,-b_height,-x_offset,track,-b_height,theta_spot,x_spot,y_spot,z_spot]




"""
Main Loop
"""

while (continuer):
        clock.tick(50)     
        
        for event in pygame.event.get(): # User did something.
            if event.type == pygame.QUIT: # Si el usuario hace clic en cerrar ventana.
                continuer = False     
            if event.type == pygame.MOUSEBUTTONDOWN: 
                mouseclick = True
            else:
                mouseclick = False
                
        for i in range (0,nj): #leer la posición del joystick analógico
            joypos[i] = joystick.get_axis(i)                        
        for i in range (0,nb):  #botones de lectura
            joybut[i] = joystick.get_button(i)
        joyhat = joystick.get_hat(0)  # read hat  
        
        """Animacion"""
        
        if (joybut[but_walk] == 0)&(joybut[but_pee] == 0)&(joybut[but_twist] == 0)&(joybut[but_sit] == 0)&(joybut[but_lie] == 0)&(joybut[but_anim] == 0)&(joybut[but_move] == 0)&(cerrar == True):
            cerrar = False
        
        #CAMINANDO
        if (joybut[but_walk] == 1)&(caminando == True)&(stop == False)&(cerrar == False): #Salir del modo caminar
            stop = True
            cerrar = True
            if (abs(t-int(t))<=tstep):
                tstop = int(t)
            else:
                tstop = int(t)+1
            
            if (t==0):
                tstop = 1
        
        if (joybut[but_walk] == 1)&(caminando == False)&(libre == True): #Entrar en modo caminar
            caminando = True
            stop = False
            libre = False
            t=0
            tstart = 1
            tstop = 1000
            cerrar = True
            trec = int(t)
            

         #SENTADO Y DANDO LA PATA
        if (joybut[but_sit] == 1)&(sitting == False)&(libre == True): #Entrar en modo sentado
            sitting = True
            stop = False
            libre = False
            t=0
            cerrar = True
            

        if (joybut[but_sit] == 1)&(sitting == True)&(stop == False)&(cerrar == False): #Salir del modo sentado
            stop = True
            cerrar = True


        #CAMBIAR DE PESO Y ORINAR
        if (joybut[but_pee] == 1)&(shifting == False)&(libre == True): #Entrar en modo animacion orinar
            shifting = True
            stop = False
            libre = False
            t=0
            cerrar = True
        
            
        if (joybut[but_pee] == 1)&(shifting == True)&(stop == False)&(cerrar == False): #Salir del modo animacion orinar
            stop = True
            cerrar = True
                       

        #ACOSTARSE
        if (joybut[but_lie] == 1)&(lying == False)&(libre == True): #Entrar en modo mintiendo
            lying = True
            stop = False
            libre = False
            t=0
            cerrar = True
            

        if (joybut[but_lie] == 1)&(lying== True)&(stop == False)&(cerrar == False): #Salir del modo mintiendo
            stop = True
            cerrar = True
            


        #RETORTIJÓN
        if (joybut[but_twist] == 1)&(twisting == False)&(libre == True): #Enter in sitting mode
            twisting = True
            libre = False
            t=0
            cerrar = True
        
        #CAMINAR
        if (caminando == True):  
            coef = 1.2
            #Establecer la dirección y velocidad de la marcha           
            #Establecer el radio de dirección
            
            if (joybut[but_move] == True)&(tstep > 0)&(cerrar == False):
                tstep = 0
                cerrar = True
                
            if (joybut[but_move] == True)&(tstep == 0)&(cerrar == False):
                tstep = tstep1
                cerrar = True    
                
            print (tstep)    
                
            if (abs(joypos[pos_leftright])>0.2)|(abs(joypos[pos_frontrear])>0.2)|(stop == True):                
                t=t+tstep
                trec = int(t)+1
                
                module_old = module
                caminando_direction_old = caminando_direction
                steering_old = steering
                
                x_old = module_old*cos(caminando_direction_old)
                y_old = module_old*sin(caminando_direction_old)
                
                #Solicitud de actualización
                module = sqrt(joypos[pos_leftright]**2 + joypos[pos_frontrear]**2)
                caminando_direction = (atan2(-joypos[pos_leftright],-joypos[pos_frontrear])%(2*pi)+pi/2)%(2*pi)
                
                x_new = module*cos(caminando_direction)
                y_new = module*sin(caminando_direction)
                                
                #Actualización de la dirección          
                if (abs(joypos[pos_turn]) < 0.2):
                    cw = 1
                    if (steering<2000):
                        steering = min(1e6,steering_old*coef) 
                    else:
                        steering = 1e6
                else:
                    steering = 2000-(abs(joypos[0])-0.2)*2000/0.8+0.001
                    if ((steering/steering_old)>coef):                       
                        steering = steering_old*coef
                    if ((steering_old/steering)>coef):                       
                        steering = steering_old/coef   
                        if (steering <0.001):
                            steering = 0.001
                    cw = -np.sign(joypos[0])
                
                
                gap = sqrt((x_new-x_old)**2+(y_new-y_old)**2)
                
                if (gap>0.01):
                    x_new = x_old+ (x_new-x_old)/gap*0.01
                    y_new = y_old+ (y_new-y_old)/gap*0.01
                    module = sqrt(x_new**2+y_new**2)
                    caminando_direction = atan2(y_new,x_new)
                                                        
                #Reduce la velocidad lateralmente y hacia atrás
                min_h_amp = h_amp*(1/2e6*steering+1/2)               
                xa = 1+cos(caminando_direction-pi/2) 
                caminando_speed = min (1, module) * min(h_amp,min_h_amp) * (1/8*xa**2+1/8*xa+1/4)                
                
                
            if ((abs(joypos[pos_leftright])<0.2)&(abs(joypos[pos_frontrear])<0.2))&(stop == False):  
                t=t+tstep                
                module = max (0, module-0.01)
                caminando_speed = module* h_amp * ((1+cos(caminando_direction-pi/2))/2*0.75+0.25)
                if (steering<2000):
                    steering = min(1e6,steering*coef) 
                else:
                    steering = 1e6
                cw=1    
                if (t>trec): 
                    t=trec

            """ 
            Si tiene una IMU que mide los valores de Ángulo[0] y Ángulo[1],
            se pueden transferir a theta_spot.
            """                
            theta_spot[3] = Angle [0] # Angulo alrededor del eje x
            theta_spot[4] = Angle [1] # Angulo alrededor del eje y
            theta_spot[0] = Angle [0] # Angulo alrededor del eje x
            theta_spot[1] = Angle [1] # Angulo alrededor del eje y
                        
            if (t< tstart):           
                caminar = ActualizarPosicion(x_offset,steering,caminando_direction,cw,caminando_speed,t,tstep,theta_spot,x_spot,y_spot,z_spot,'start')
                pos = caminar.actualizar_posicion()
            else:
                if (t<tstop):
                    caminar = ActualizarPosicion(x_offset,steering,caminando_direction,cw,caminando_speed,t,tstep,theta_spot,x_spot,y_spot,z_spot,'walk')
                    pos = caminar.actualizar_posicion()                    
                else:
                    caminar = ActualizarPosicion(x_offset,steering,caminando_direction,cw,caminando_speed,t,tstep,theta_spot,x_spot,y_spot,z_spot,'stop')
                    pos = caminar.actualizar_posicion()
                        
                            
            theta_spot = pos[12]
            x_spot = pos[13]
            y_spot = pos[14]                 
            z_spot = pos[15]
            
            
            if (t>(tstop+1-tstep)):
                stop = False
                caminando = False
                libre = True
        
                
        

        if (sitting == True):
            alpha_sitting = -30/180*pi 
            alpha_pawing = 0/180*pi
            L_paw = 220
            
            x_end_sitting = xlr- L2 + L1*cos(pi/3) + Lb/2*cos(-alpha_sitting) - d*sin (-alpha_sitting)
            z_end_sitting = L1*sin(pi/3)+ Lb/2*sin(-alpha_sitting) + d*cos(-alpha_sitting)
            start_frame_pos = [0,0,0,x_offset,0,b_height] # rotaciones x,y,z y luego traslaciones

            #end_frame_pos = [0,0,0,x_offset,0,b_height-20] # x,y,z rotations then translations
            end_frame_pos = [0,alpha_sitting,0, x_end_sitting,0,z_end_sitting] # x,y,z rotations then translations
            pos = moving (t, start_frame_pos,end_frame_pos, pos)
            
            if (t==1)&(pawing == False):
                pos_sit_init = pos
            
            if (t == 1): # Es posible manosear
                if (pawing == True):
                    #print (pos_sit_init[3],pos_sit_init[5])
                    pos[3] = pos_sit_init[3]+ (L_paw*cos(alpha_pawing)-pos_sit_init[3])*(joypar+1)/2
                    pos[5] = pos_sit_init[5]+ (-d-L_paw*sin(alpha_pawing)-pos_sit_init[5])*(joypar+1)/2
                    
                    pos[0] = pos_sit_init[0]+ (L_paw*cos(alpha_pawing)-pos_sit_init[0])*(joypal+1)/2
                    pos[2] = pos_sit_init[2]+ (-d-L_paw*sin(alpha_pawing)-pos_sit_init[2])*(joypal+1)/2
                    
                    thetarf = IK(pos[3], pos[4], pos[5], -1)[0]
                    thetalf = IK(pos[0], pos[1], pos[2], -1)[0]
                    #Actualización de la posición absoluta de la pata delantera derecha
                    legrf = FK(thetarf,-1) 
                    leglf = FK(thetalf,-1) 
                    xlegrf =xrf+pos[3]
                    ylegrf =yrf+pos[4]
                    zlegrf =pos[5]
                    xleglf =xlf+pos[0]
                    yleglf =ylf+pos[1]
                    zleglf =pos[2]
                    
                    theta_spot_sit = pos[12]
                    
                    x_spot_sit = pos[13]
                    y_spot_sit = pos[14]
                    z_spot_sit = pos[15]
                    
                    M = xyz_rotation_matrix(theta_spot_sit[3],theta_spot_sit[4],theta_spot_sit[2]+theta_spot_sit[5],False)
                    
                    paw_rf = new_coordinates(M,xlegrf,ylegrf,zlegrf,x_spot_sit[1],y_spot_sit[1],z_spot_sit[1])
                    paw_lf = new_coordinates(M,xleglf,yleglf,zleglf,x_spot_sit[1],y_spot_sit[1],z_spot_sit[1])
                    
                    x_spot_sit[3] = paw_rf[0]
                    y_spot_sit[3] = paw_rf[1]
                    z_spot_sit[3] = paw_rf[2]
                    x_spot_sit[2] = paw_lf[0]
                    y_spot_sit[2] = paw_lf[1]
                    z_spot_sit[2] = paw_lf[2]
                    
                    
                    pos[13] = x_spot_sit
                    pos[14] = y_spot_sit
                    pos[15] = z_spot_sit
                    
                joypar_old = joypar    
                if (joypal == -1):
                    if (((joypos[pos_rightpaw] != 0)&(joypos[pos_rightpaw] !=-1))|(joypar != -1)):
                        pawing = True
                        if (joypos[pos_rightpaw]>= joypar):
                             joypar = min (joypos[pos_rightpaw],joypar+0.05)
                        else:
                             joypar = max (joypos[pos_rightpaw],joypar-0.05)
                    else:
                        pawing = False     
                    
                if (joypar_old == -1):
                    if (((joypos[pos_leftpaw] != 0)&(joypos[pos_leftpaw] !=-1))|(joypal != -1)):
                        pawing = True
                        if (joypos[pos_leftpaw]>= joypal):
                             joypal = min (joypos[pos_leftpaw],joypal+0.05)
                        else:
                             joypal = max (joypos[pos_leftpaw],joypal-0.05)
                    else:
                        pawing = False         
                    

            if (stop == False):
                t=t+4*tstep
                if (t>=1):
                    t= 1
            elif (pawing == False):
                t=t-4*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    sitting = False
                    libre = True

        if (shifting == True):                        
            x_end_shifting = ra_longi
            y_end_shifting = -ra_lat
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            end_frame_pos = [0,0,0, x_end_shifting+x_offset,y_end_shifting,b_height] # x,y,z rotations then translations
            pos = moving (t, start_frame_pos,end_frame_pos, pos)
            
            if (t==1)&(peeing == False):
                pos_shift_init = pos
            
            if (t == 1): # Es posible levantar la pierna izquierda.
                
                if (peeing == True): 
                    pos[9] = pos_shift_init[9]+ (0-pos_shift_init[9])*(joype+1)/2
                    pos[10] = pos_shift_init[10]+ (130-pos_shift_init[10])*(joype+1)/2
                    pos[11] = pos_shift_init[11]+ (-20-pos_shift_init[11])*(joype+1)/2
                    
                    thetalr = IK(pos[9], pos[10], pos[11], 1)[0]
                    # Actualización de la posición absoluta de la pata trasera izquierda
                    leglr = FK(thetalr,1)                    
                    xleglr =xlr+pos[9]
                    yleglr =ylr+pos[10]
                    zleglr =pos[11]
                    theta_spot_shift = pos[12]
                    x_spot_shift = pos[13]
                    y_spot_shift = pos[14]
                    z_spot_shift = pos[15]
                    M = xyz_rotation_matrix(theta_spot_shift[3],theta_spot_shift[4],theta_spot_shift[2]+theta_spot_shift[5],False)
                    pee_lr = new_coordinates(M,xleglr,yleglr,zleglr,x_spot_shift[1],y_spot_shift[1],z_spot_shift[1])
                    
                    x_spot_shift[5] = pee_lr[0]
                    y_spot_shift[5] = pee_lr[1]
                    z_spot_shift[5] = pee_lr[2]
                    pos[13] = x_spot_shift
                    pos[14] = y_spot_shift
                    pos[15] = z_spot_shift
                    
                if ((joypos[pos_leftpaw] != 0)&(joypos[pos_leftpaw] !=-1))|(joype != -1):
                    peeing = True
                    if (joypos[pos_leftpaw]>= joype):
                         joype = min (joypos[pos_leftpaw],joype+0.1)
                    else:
                         joype = max (joypos[pos_leftpaw],joype-0.1)
                else:
                    peeing = False
                
            if (stop == False):
                t=t+4*tstep
                if (t>=1):
                    t= 1
            elif (peeing == False):
                t=t-4*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    shifting = False
                    libre = True

        
        if (lying == True):
            angle_lying = 40/180*pi
            x_end_lying= xlr - L2 + L1*cos(angle_lying)+ Lb/2
            z_end_lying = L1*sin(angle_lying)+ d
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            end_frame_pos = [0,0,0, x_end_lying,0,z_end_lying] # x,y,z rotations then translations
            pos = moving (t, start_frame_pos,end_frame_pos, pos)
            if (stop == False):
                t=t+3*tstep
                if (t>=1):
                    t= 1
            else:
                t=t-3*tstep
                if (t<= 0):
                    t= 0
                    stop  = False
                    lying = False
                    libre = True
        
        if (twisting == True):
            x_angle_twisting = 0/180*pi
            y_angle_twisting = 0/180*pi
            z_angle_twisting = 30/180*pi
            start_frame_pos = [0,0,0,x_offset,0,b_height] # x,y,z rotations then translations
            
            t=t+4*tstep
            if (t>=1):
               t=1
               twisting = False
               libre = True
                
            if (t<0.25):
                end_frame_pos = [x_angle_twisting,y_angle_twisting,z_angle_twisting, x_offset,0,b_height] # x,y,z rotations then translations
                pos = moving (t*4, start_frame_pos,end_frame_pos, pos)                
            if (t>=0.25)&(t<0.5):
                end_frame_pos = [x_angle_twisting,y_angle_twisting,z_angle_twisting, x_offset,0,b_height] # x,y,z rotations then translations
                pos = moving ((t-0.25)*4, end_frame_pos,start_frame_pos, pos)  
            if (t>=0.5)&(t<0.75):
                end_frame_pos = [-x_angle_twisting,-y_angle_twisting,-z_angle_twisting, x_offset,0,b_height]
                pos = moving ((t-0.5)*4, start_frame_pos,end_frame_pos, pos) 
            if (t>=0.75)&(t<=1):
                end_frame_pos = [-x_angle_twisting,-y_angle_twisting,-z_angle_twisting, x_offset,0,b_height]
                pos = moving ((t-0.75)*4, end_frame_pos,start_frame_pos, pos)     
             
            
        xc = steering* cos(caminando_direction)
        yc = steering* sin(caminando_direction)
            
        center_x = x_spot[0]+(xc*cos(theta_spot[2])-yc*sin(theta_spot[2])) # Posición x del centro absoluto
        center_y = y_spot[0]+(xc*sin(theta_spot[2])+yc*cos(theta_spot[2])) # Posición y del centro absoluto


        
        thetalf = IK(pos[0], pos[1], pos[2], 1)[0]
        thetarf = IK(pos[3], pos[4], pos[5], -1)[0]
        thetarr = IK(pos[6], pos[7], pos[8], -1)[0]
        thetalr = IK(pos[9], pos[10], pos[11], 1)[0]
        
        """
        ************************************************************************************************
        
        thetalf, theatre, theatre, theatre son los conjuntos de ángulos que se pueden enviar a los servos
        para generar el movimiento de Spotmicro
        
        Aquí es donde se puede realizar la llamada a la función de movimiento
        del servo. La función de movimiento depende del tipo de servos
        y de los controladores que se utilizan para controlarlos.
        
        -Shields I2c, generadores PWM
        -Carrera máxima de servos 180°, 270°, 360°...
           
        Las posiciones cero y las carreras de los servos deben ajustarse
        
        ************************************************************************************************ 
        """
        

 
        stance = [False, False, False, False]
        if (pos[15][2] < 0.01):            
            stance[0] = True
        if (pos[15][3] < 0.01):           
            stance[1] = True
        if (pos[15][4] < 0.01):           
            stance[2] = True
        if (pos[15][5] < 0.01):              
            stance[3] = True
        
        
        SpotAnim.animate(pos,t,pi/12,-135/180*pi,Angle,center_x,center_y,
                         thetalf,thetarf,thetarr,thetalr,caminando_speed,caminando_direction,steering,stance)
        #SpotAnim.animate(pos,t,pi/2,-0/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,caminando_speed,caminando_direction,steering,stance)
        #SpotAnim.animate(pos,t,0,-0/180*pi,Angle,center_x,center_y,thetalf,thetarf,thetarr,thetalr,caminando_speed,caminando_direction,steering,stance)

        pygame.display.flip()
        if (libre == True):
            sleep(0.1)
        
        """ Actualizar CG """
        CG = SpotCG.CG_calculation (thetalf,thetarf,thetarr,thetalr)
        # Cálculo de la posición absoluta del CG
        M = xyz_rotation_matrix(theta_spot[0],theta_spot[1],theta_spot[2],False)
        CGabs = new_coordinates(M,CG[0],CG[1],CG[2],x_spot[1],y_spot[1],z_spot[1])
        dCG = SpotCG.CG_distance(x_spot[2:6],y_spot[2:6],z_spot[2:6],CGabs[0],CGabs[1],stance)
        
        
        pos[13][6] = CG[0] #x
        pos[14][6] = CG[1] #y
        pos[15][6] = CG[2] #z
        
        pos[13][7] = CGabs[0] #x
        pos[14][7] = CGabs[1] #y
        pos[15][7] = CGabs[2] #z
        
        pos[13][8] = dCG[1] #xint
        pos[14][8] = dCG[2] #yint
        pos[15][8] = dCG[3] #balance
        
        distance.append(dCG[0])
        timing.append(t) 
                                          
pygame.quit()



            