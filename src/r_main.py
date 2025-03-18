import pygame
import numpy as np
from math import pi, sin, cos, atan, atan2, sqrt
from config.puntos_torso import *
from config.dimensiones import L1, L2, Lb, d
from config.parametros_caminar import tstep, x_offset, track, b_height, but_walk, but_sit, but_pee, but_lie, but_twist, pos_frontrear, pos_leftright, steering, cw, caminando_speed, caminando_direction, nb, nj, stance, Angle
from utils.operador import xyz_rotation_matrix, new_coordinates
from motion.move import moving
from utils.cinematica import IK, FK
from src.motion.ActualizarPosicion import ActualizarPosicion
from centro_gravedad.centro_g import SpotCG
from animation.animacion import SpotAnime

SpotCG = SpotCG()
SpotAnime = SpotAnime()

class SpotMicro:
    def __init__(self):
        # Inicialización de variables
        self.caminando = False # Activacion para la secuencia de activacion
        self.sitting = False
        self.shifting = False
        self.lying = False
        self.twisting = False
        self.pawing = False
        self.peeing = False
        self.libre = True #Spot está listo para recibir nuevo comando
        self.stop = False
        #self.cerrar = False # Bloqueo del recorrido de la tecla/botón como "filtro de rebote"
        self.t = 0
        self.tstep = tstep
        self.theta_spot = [0, 0, 0, 0, 0, 0]
        self.x_spot = [0, x_offset, xlf, xrf, xrr, xlr, 0, 0, 0]
        self.y_spot = [0, 0, ylf + track, yrf - track, yrr - track, ylr + track, 0, 0, 0]
        self.z_spot = [0, b_height, 0, 0, 0, 0, 0, 0, 0]
        self.pos = self.init_positions()
        self.CG = None
        self.CGabs = None
        self.dCG = None

    def init_positions(self):
        """Inicializa las posiciones iniciales de las patas."""
        pos_init = [-x_offset, track, -b_height, -x_offset, -track, -b_height,
                    -x_offset, -track, -b_height, -x_offset, track, -b_height]
        thetarf = IK(pos_init[3], pos_init[4], pos_init[5], -1)[0]
        thetalf = IK(pos_init[0], pos_init[1], pos_init[2], 1)[0]
        thetarr = IK(pos_init[6], pos_init[7], pos_init[8], -1)[0]
        thetalr = IK(pos_init[9], pos_init[10], pos_init[11], 1)[0]
        
        self.CG = SpotCG.CG_calculation(thetalf, thetarf, thetarr, thetalr)
        M = xyz_rotation_matrix(self.theta_spot[0],self.theta_spot[1],self.theta_spot[2],False)
        self.CGabs = new_coordinates(M,self.CG[0],self.CG[1],self.CG[2],self.x_spot[1],self.y_spot[1],self.z_spot[1])
        self.dCG = SpotCG.CG_distance(self.x_spot[2:6],self.y_spot[2:6],self.z_spot[2:6],self.CGabs[0],self.CGabs[1],stance)
        
        self.x_spot = [0, x_offset, xlf, xrf, xrr, xlr,self.CG[0],self.CGabs[0],self.dCG[1]]
        self.y_spot = [0,0, ylf+track, yrf-track, yrr-track, ylr+track,self.CG[1],self.CGabs[1],self.dCG[2]]
        self.z_spot = [0,b_height,0,0,0,0,self.CG[2],self.CGabs[2],self.dCG[3]]
        
        pos = [-x_offset, track, -b_height, -x_offset, -track, -b_height,
               -x_offset, -track, -b_height, -x_offset, track, -b_height,
               self.theta_spot, self.x_spot, self.y_spot, self.z_spot]
        
        return pos

    def handle_joystick_events(self, joypos, joybut): 
        """Maneja los eventos del joystick."""
        if joybut[but_walk] == 0:
            self.toggle_lock()
        if joybut[but_walk] == 1:
            self.toggle_walking()
        if joybut[but_sit] == 1:
            self.toggle_sitting()
        if joybut[but_pee] == 1:
            self.toggle_shifting()
        if joybut[but_lie] == 1:
            self.toggle_lying()
        if joybut[but_twist] == 1:
            self.toggle_twisting()
            
    def toggle_lock(self):
        """Controla el modo bloqueo"""
        pass

    def toggle_walking(self):
        """Controla el modo de caminar."""
        if not self.caminando and self.libre:
            self.caminando = True
            self.libre = False
            self.t = 0
        elif self.caminando and not self.stop:
            self.stop = True

    def toggle_sitting(self):
        """Controla el modo de sentarse."""
        if not self.sitting and self.libre:
            self.sitting = True
            self.libre = False
            self.t = 0
        elif self.sitting and not self.stop:
            self.stop = True

    def toggle_shifting(self):
        """Controla el modo de cambiar de peso."""
        if not self.shifting and self.libre:
            self.shifting = True
            self.libre = False
            self.t = 0
        elif self.shifting and not self.stop:
            self.stop = True

    def toggle_lying(self):
        """Controla el modo de acostarse."""
        if not self.lying and self.libre:
            self.lying = True
            self.libre = False
            self.t = 0
        elif self.lying and not self.stop:
            self.stop = True

    def toggle_twisting(self):
        """Controla el modo de retorcerse."""
        if not self.twisting and self.libre:
            self.twisting = True
            self.libre = False
            self.t = 0
        elif self.twisting and not self.stop:
            self.stop = True

    def walk(self, joypos):
        """Lógica para caminar."""
        if abs(joypos[pos_leftright]) > 0.2 or abs(joypos[pos_frontrear]) > 0.2 or self.stop:
            self.t += self.tstep
            module = sqrt(joypos[pos_leftright]**2 + joypos[pos_frontrear]**2)
            direction = atan2(-joypos[pos_leftright], -joypos[pos_frontrear])
            # Actualizar posición y dirección
            self.pos = self.update_position(module, direction)
            

    def update_position(self, module, direction):
        """Actualiza la posición del robot mientras camina."""
        caminar = ActualizarPosicion(x_offset, steering, direction, cw, caminando_speed, self.t, self.tstep,
                                     self.theta_spot, self.x_spot, self.y_spot, self.z_spot, 'walk')
        return caminar.actualizar_posicion()

    def sit(self):
        """Lógica para sentarse."""
        alpha_sitting = -30 / 180 * pi
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]
        end_frame_pos = [0, alpha_sitting, 0, x_end_sitting, 0, z_end_sitting]
        self.pos = moving(self.t, start_frame_pos, end_frame_pos, self.pos)

    def shift_weight(self):
        """Lógica para cambiar de peso."""
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]
        end_frame_pos = [0, 0, 0, x_end_shifting + x_offset, y_end_shifting, b_height]
        self.pos = moving(self.t, start_frame_pos, end_frame_pos, self.pos)

    def lie_down(self):
        """Lógica para acostarse."""
        angle_lying = 40 / 180 * pi
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]
        end_frame_pos = [0, 0, 0, x_end_lying, 0, z_end_lying]
        self.pos = moving(self.t, start_frame_pos, end_frame_pos, self.pos)

    def twist(self):
        """Lógica para retorcerse."""
        x_angle_twisting = 0 / 180 * pi
        y_angle_twisting = 0 / 180 * pi
        z_angle_twisting = 30 / 180 * pi
        start_frame_pos = [0, 0, 0, x_offset, 0, b_height]
        self.t += 4 * self.tstep
        if self.t >= 1:
            self.t = 1
            self.twisting = False
            self.libre = True

    def animate(self):
        """Animación del robot."""
        xc = steering * cos(caminando_direction)
        yc = steering * sin(caminando_direction)
        center_x = self.x_spot[0] + (xc * cos(self.theta_spot[2]) - yc * sin(self.theta_spot[2]))
        center_y = self.y_spot[0] + (xc * sin(self.theta_spot[2]) + yc * cos(self.theta_spot[2]))
        thetalf = IK(self.pos[0], self.pos[1], self.pos[2], 1)[0]
        thetarf = IK(self.pos[3], self.pos[4], self.pos[5], -1)[0]
        thetarr = IK(self.pos[6], self.pos[7], self.pos[8], -1)[0]
        thetalr = IK(self.pos[9], self.pos[10], self.pos[11], 1)[0]
        stance = [False, False, False, False]
        for i in range(4):
            if self.pos[15][i + 2] < 0.01:
                stance[i] = True
        SpotAnime.animate(self.pos, self.t, Angle, center_x, center_y,
                         thetalf, thetarf, thetarr, thetalr, caminando_speed, caminando_direction, steering, stance)

    def update_cg(self):
        """Actualiza el centro de gravedad."""
        self.CG = SpotCG.CG_calculation(*[IK(*p)[0] for p in zip(self.pos[::3], self.pos[1::3], self.pos[2::3], [1, -1, -1, 1])])
        M = xyz_rotation_matrix(*self.theta_spot[:3], False)
        self.CGabs = new_coordinates(M, self.CG[0], self.CG[1], self.CG[2], self.x_spot[1], self.y_spot[1], self.z_spot[1])
        self.dCG = SpotCG.CG_distance(self.x_spot[2:6], self.y_spot[2:6], self.z_spot[2:6], self.CGabs[0], self.CGabs[1], stance)


def main():
    
    pygame.init()
    #screen = pygame.display.set_mode((600, 600))
    pygame.display.set_caption("ROBOT")
    pygame.joystick.init() # Inicializar el modulo del joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    clock = pygame.time.Clock()

    spot_micro = SpotMicro()
    
    running = True

    while running:
        clock.tick(50)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        joypos = [joystick.get_axis(i) for i in range(nj)]
        joybut = [joystick.get_button(i) for i in range(nb)]

        spot_micro.handle_joystick_events(joypos, joybut)

        if spot_micro.caminando:
            spot_micro.walk(joypos)
        #elif spot_micro.sitting:
        #    spot_micro.sit()
        #elif spot_micro.shifting:
        #    spot_micro.shift_weight()
        #elif spot_micro.lying:
        #    spot_micro.lie_down()
        #elif spot_micro.twisting:
        #    spot_micro.twist()

        spot_micro.animate()
        spot_micro.update_cg()

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()