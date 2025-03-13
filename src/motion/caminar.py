from src.motion.ActualizarPosicion import ActualizarPosicion
        
def start_walk_stop(x_offset,steering,walking_direction,cw,h_amp,t,tstep,theta_spot,x_spot,y_spot,z_spot,step_phase):
    control = ActualizarPosicion(x_offset, steering, walking_direction, step_phase, tstep, cw, h_amp, t, theta_spot, x_spot, y_spot, z_spot)
    pos = control.actualizar_posicion()    
    return pos