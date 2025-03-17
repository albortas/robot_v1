import pygame
import numpy as np
from math import pi, sin, cos, atan, atan2, sqrt
from config.puntos_torso import *
from config.dimensiones import L1, L2, Lb, d
from config.parametros_caminar import *
from utils.operador import xyz_rotation_matrix, new_coordinates
from motion.move import moving
from utils.cinematica import IK, FK
from src.motion.ActualizarPosicion import ActualizarPosicion
from centro_gravedad.centro_g import SpotCG
from animation.animation import SpotAnime

class Controller:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((600, 600))
        pygame.display.set_caption("SPOTMICRO")
        self.joystick = self.init_joystick()
        self.clock = pygame.time.Clock()
        self.spot_cg = SpotCG()
        self.spot_anim = SpotAnime()

        # Inicialización de variables
        self.init_variables()

    def init_joystick(self):
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        return joystick

    def init_variables(self):
        self.nj = 6  # Número de joysticks
        self.nb = 10  # Número de botones
        self.joypos = np.zeros(self.nj)
        self.joybut = np.zeros(self.nb)

        self.continuer = True
        self.t = 0
        self.tstart = 1
        self.tstop = 1000
        self.tstep = 0.01
        self.tstep1 = self.tstep

        self.states = {
            "free": True,
            "sitting": False,
            "walking": False,
            "lying": False,
            "twisting": False,
            "pawing": False,
            "shifting": False,
            "peeing": False,
            "stop": False,
            "lock": False,
            "lockmouse": False,
            "mouseclick": False,
        }

        self.walking_params = {
            "speed": 0,
            "direction": 0,
            "steering": 1e6,
            "module": 0,
        }

        self.joystick_values = {
            "peeing": -1,
            "pawing_right": -1,
            "pawing_left": -1,
        }

        self.positions = {
            "x_spot": [0, x_offset, xlf, xrf, xrr, xlr, 0, 0, 0],
            "y_spot": [0, 0, ylf + track, yrf - track, yrr - track, ylr + track, 0, 0, 0],
            "z_spot": [0, b_height, 0, 0, 0, 0, 0, 0, 0],
        }

        self.theta_spot = [0, 0, 0, 0, 0, 0]  # Ángulos del cuerpo
        self.pos_init = [
            -x_offset, track, -b_height,
            -x_offset, -track, -b_height,
            -x_offset, -track, -b_height,
            -x_offset, track, -b_height
        ]

        self.calculate_initial_positions()

    def calculate_initial_positions(self):
        thetarf = IK(self.pos_init[3], self.pos_init[4], self.pos_init[5], -1)[0]
        thetalf = IK(self.pos_init[0], self.pos_init[1], self.pos_init[2], 1)[0]
        thetarr = IK(self.pos_init[6], self.pos_init[7], self.pos_init[8], -1)[0]
        thetalr = IK(self.pos_init[9], self.pos_init[10], self.pos_init[11], 1)[0]

        CG = self.spot_cg.CG_calculation(thetalf, thetarf, thetarr, thetalr)
        M = xyz_rotation_matrix(self.theta_spot[0], self.theta_spot[1], self.theta_spot[2], False)
        CGabs = new_coordinates(M, CG[0], CG[1], CG[2], self.positions["x_spot"][1], self.positions["y_spot"][1], self.positions["z_spot"][1])
        dCG = self.spot_cg.CG_distance(self.positions["x_spot"][2:6], self.positions["y_spot"][2:6], self.positions["z_spot"][2:6], CGabs[0], CGabs[1], stance)

        self.positions["x_spot"] = [0, x_offset, xlf, xrf, xrr, xlr, CG[0], CGabs[0], dCG[1]]
        self.positions["y_spot"] = [0, 0, ylf + track, yrf - track, yrr - track, ylr + track, CG[1], CGabs[1], dCG[2]]
        self.positions["z_spot"] = [0, b_height, 0, 0, 0, 0, CG[2], CGabs[2], dCG[3]]

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.continuer = False
            if event.type == pygame.MOUSEBUTTONDOWN:
                self.states["mouseclick"] = True
            else:
                self.states["mouseclick"] = False

        for i in range(self.nj):
            self.joypos[i] = self.joystick.get_axis(i)
        for i in range(self.nb):
            self.joybut[i] = self.joystick.get_button(i)

    def update_walking(self):
        if self.states["walking"]:
            if self.joybut[but_walk] == 1 and not self.states["lock"]:
                self.states["stop"] = True
                self.states["lock"] = True
                self.tstop = int(self.t) if abs(self.t - int(self.t)) <= self.tstep else int(self.t) + 1
                if self.t == 0:
                    self.tstop = 1

            if self.joybut[but_walk] == 1 and not self.states["walking"] and self.states["free"]:
                self.states["walking"] = True
                self.states["stop"] = False
                self.states["free"] = False
                self.t = 0
                self.tstart = 1
                self.tstop = 1000
                self.states["lock"] = True

    def update_sitting(self):
        if self.joybut[but_sit] == 1 and not self.states["sitting"] and self.states["free"]:
            self.states["sitting"] = True
            self.states["stop"] = False
            self.states["free"] = False
            self.t = 0
            self.states["lock"] = True

        if self.joybut[but_sit] == 1 and self.states["sitting"] and not self.states["stop"] and not self.states["lock"]:
            self.states["stop"] = True
            self.states["lock"] = True

    def update_other_states(self):
        # Similar a `update_sitting`, implementar para lying, twisting, etc.
        pass

    def main_loop(self):
        while self.continuer:
            self.clock.tick(50)
            self.handle_events()
            self.update_walking()
            self.update_sitting()
            self.update_other_states()

            # Renderizar animación
            self.spot_anim.animate(
                pos=self.positions,
                t=self.t,
                angle_x=pi / 12,
                angle_y=-135 / 180 * pi,
                angle_z=0,
                center_x=self.positions["x_spot"][0],
                center_y=self.positions["y_spot"][0],
                thetalf=IK(self.positions["pos_init"][0], self.positions["pos_init"][1], self.positions["pos_init"][2], 1)[0],
                thetarf=IK(self.positions["pos_init"][3], self.positions["pos_init"][4], self.positions["pos_init"][5], -1)[0],
                thetarr=IK(self.positions["pos_init"][6], self.positions["pos_init"][7], self.positions["pos_init"][8], -1)[0],
                thetalr=IK(self.positions["pos_init"][9], self.positions["pos_init"][10], self.positions["pos_init"][11], 1)[0],
                walking_speed=self.walking_params["speed"],
                walking_direction=self.walking_params["direction"],
                steering=self.walking_params["steering"],
                stance=[False, False, False, False]
            )

            pygame.display.flip()

        pygame.quit()


if __name__ == "__main__":
    controller = Controller()
    controller.main_loop()