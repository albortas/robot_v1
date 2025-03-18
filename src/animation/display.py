import numpy as np
from src.utils.operador import xyz_rotation_matrix, new_coordinates

conv = 0

def display_rotate(x_spot,y_spot,z_spot,theta_spot,thetax,thetaz,xl,yl,zl):
    line = []
    Ma = xyz_rotation_matrix(theta_spot[3],theta_spot[4],theta_spot[2]+theta_spot[5],False)
    Mb = xyz_rotation_matrix(theta_spot[0],theta_spot[1],0,False)
    M1 = xyz_rotation_matrix(thetax,0,thetaz,True)
    
    for i in range (0,len(xl)):    
        # Coordenadas absolutas de las líneas puntuales en los marcos x,y,z
        out0 = new_coordinates(Ma,xl[i],yl[i],zl[i],x_spot,y_spot,z_spot)
        out = new_coordinates(Mb,out0[0],out0[1],out0[2],0,0,0)
        
        #Coordenadas para visualizar en pantalla
        disp = new_coordinates(M1,out[0],out[1],out[2],0,0,0)
        
        yd= disp[1]
        factor_escala = 2**(yd*conv/2000)
        xd= disp[0] / factor_escala
        zd= disp[2] / factor_escala
        
        line.append([int(300+xd),int(300-zd)])
    return line