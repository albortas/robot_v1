from math import pi, sin, cos
import pygame
import numpy as np
import src.config.puntos_torso as torso
import src.config.colores as color
import src.config.coordenadas as coor
from src.utils.cinematica import FK
from src.utils.operador import rotar_display


class SpotAnime:
    
    def __init__(self):
        self.screen = pygame.display.set_mode((600, 600))
        self.lineas_leg = np.array(range(40)).reshape(4,5,2)
    
    def draw_floor(self, pos):
        """Dibuja el suelo y la cuadrícula."""
        line = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                              [pos[12][0], pos[12][1], 0, 0, 0, 0],
                              coor.xFloor, coor.yFloor, coor.zFloor)
        pygame.draw.polygon(self.screen, color.GREY, line, 0)

        for i in range(11):
            grid_line_x = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [pos[12][0], pos[12][1], 0, 0, 0, 0],
                                         [-500 + i * 100, -500 + i * 100],
                                         [-500, 500], [0, 0])
            pygame.draw.lines(self.screen, color.DARK_GREY, False, grid_line_x, 1)

            grid_line_y = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [pos[12][0], pos[12][1], 0, 0, 0, 0],
                                         [-500, 500],
                                         [-500 + i * 100, -500 + i * 100], [0, 0])
            pygame.draw.lines(self.screen, color.DARK_GREY, False, grid_line_y, 1)

    def draw_axes(self, pos):
        """Dibuja los ejes X, Y, Z."""
        axes = {
            "X": (coor.xX, coor.yX, coor.zX, color.RED),
            "Y": (coor.xY, coor.yY, coor.zY, color.GREEN),
            "Z": (coor.xZ, coor.yZ, coor.zZ, color.BLUE)
        }
        for axis, (x, y, z, color_axis) in axes.items():
            line = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                  [0, 0, 0, 0, 0, 0], x, y, z)
            pygame.draw.lines(self.screen, color_axis, False, line, 2)
            
    

    def draw_radius_and_direction(self, pos, center_x, center_y, steering, walking_speed, walking_direction):
        """Dibuja el radio y la dirección."""
        center_display = True
        if steering < 2000:
            line_radius = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [0, 0, 0, 0, 0, 0],
                                         [center_x, pos[13][0]], [center_y, pos[14][0]], [0, 0])
        else:
            center_x1 = pos[13][0] + (center_x - pos[13][0]) / steering * 2000
            center_y1 = pos[14][0] + (center_y - pos[14][0]) / steering * 2000
            line_radius = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [0, 0, 0, 0, 0, 0],
                                         [center_x1, pos[13][0]], [center_y1, pos[14][0]], [0, 0])
            center_display = False
        
        xd = pos[13][0] + walking_speed * cos(pos[12][2] + walking_direction - pi / 2)
        yd = pos[14][0] + walking_speed * sin(pos[12][2] + walking_direction - pi / 2)
        line_direction = rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                        [0, 0, 0, 0, 0, 0],
                                        [xd, pos[13][0]], [yd, pos[14][0]], [0, 0])
        
        

        pygame.draw.lines(self.screen, color.CYAN, False, line_radius, 2)
        if (center_display == True):
            pygame.draw.circle(self.screen,color.BLACK,line_radius[0],5)
        pygame.draw.circle(self.screen,color.DARK_CYAN,line_radius[1],5)
        pygame.draw.lines(self.screen, color.GREEN, False, line_direction, 2)
        
    

    def draw_legs(self, pos, theta_spot, angles):
        """Dibuja las patas del robot."""
        lista = list(range(4))
        
        lineb = rotar_display (pos[13][1] - pos[13][0], pos[14][1] - pos[14][0], pos[15][1] - pos[15][0],theta_spot,
                                [torso.xlf,torso.xrf,torso.xrr,torso.xlr,torso.xlf],
                                [torso.ylf,torso.yrf,torso.yrr,torso.ylr,torso.ylf],
                                [torso.zlf,torso.zrf,torso.zrr,torso.zlr,torso.zlf])
        
        legs = {
            "lf": {"x": torso.xlf, "y": torso.ylf, "angles": angles[0], "n_pos": 0},
            "rf": {"x": torso.xrf, "y": torso.yrf, "angles": angles[1], "n_pos": 3},
            "rr": {"x": torso.xrr, "y": torso.yrr, "angles": angles[2], "n_pos": 6},
            "lr": {"x": torso.xlr, "y": torso.ylr, "angles": angles[3], "n_pos": 9}
        }

        for i, (leg, data) in enumerate(legs.items()):
            fk = FK(data["angles"], 1 if leg in ["lf", "lr"] else -1)
            x_leg = [data["x"], data["x"] + fk[0], data["x"] + fk[1], data["x"] + fk[2], data["x"] + pos[0 + data["n_pos"]]]
            y_leg = [data["y"], data["y"] + fk[3], data["y"] + fk[4], data["y"] + fk[5], data["y"] + pos[1 + data["n_pos"]]]
            z_leg = [0, fk[6], fk[7], fk[8], pos[2 + data["n_pos"]]]
            
            line_leg = rotar_display(pos[13][1] - pos[13][0], pos[14][1] - pos[14][0], pos[15][1] - pos[15][0],
                                      theta_spot, x_leg, y_leg, z_leg)
            lista[i] = line_leg            
            
            pygame.draw.lines(self.screen, color.RED, False, line_leg, 4)
        
        pygame.draw.lines(self.screen,color.BLUE,False,lineb,10)
           
        self.lineas_leg = np.array(lista)
        
    def draw_area_sustentacion(self, stance, pos):
        linesus = []
        for i, valor in enumerate(stance):
            if valor:
                linesus.append(self.lineas_leg[i][4])
        
        lineCG = rotar_display(-pos[13][0],-pos[14][0],-pos[15][0],[0,0,0,0,0,0],[pos[13][7],pos[13][7]],[pos[14][7],pos[14][7]],[0,pos[15][7]])
        linedCG = rotar_display(-pos[13][0],-pos[14][0],-pos[15][0],[0,0,0,0,0,0], [pos[13][7],pos[13][8]],[pos[14][7],pos[14][8]],[0,0])
        
        pygame.draw.polygon(self.screen,color.VIOLET,linesus,0)
        pygame.draw.lines(self.screen,color.BLACK,True,linesus,1)
        pygame.draw.lines(self.screen,color.BLACK,False, linedCG,1)
        pygame.draw.lines(self.screen,color.BLACK,False, lineCG,1)
        if (pos[15][8] == True):
            pygame.draw.circle(self.screen,color.GREEN,lineCG[0],3)
        else:
            pygame.draw.circle(self.screen,color.RED,lineCG[0],3)
        pygame.draw.circle(self.screen,color.DARK_GREY,lineCG[1],10)
            

    def animate(self, pos, t, angle, center_x, center_y, thetalf, thetarf, thetarr, thetalr, walking_speed, walking_direction, steering, stance):
        """Animación principal."""
        self.screen.fill(color.WHITE)

        # Dibujar elementos estáticos
        self.draw_floor(pos)
        self.draw_axes(pos)

        self.draw_area_sustentacion(stance, pos)
        # Dibujar radio y dirección
        # Dibujar patas
        self.draw_radius_and_direction(pos, center_x, center_y, steering, walking_speed, walking_direction)
        angles = [thetalf, thetarf, thetarr, thetalr]
        self.draw_legs(pos, pos[12], angles)

        pygame.display.flip()
