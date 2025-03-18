from math import pi, sin, cos
import pygame
import numpy as np
import src.config.puntos_torso as torso
import src.config.colores as color
import src.config.coordenadas as coor
from src.utils.cinematica import FK
from src.utils.operador import xyz_rotation_matrix, new_coordinates, matriz_2d


class SpotAnime:
    def __init__(self):
        self.screen = pygame.display.set_mode((600, 600))
        self.conv = 0
        self.escala = 1
        self.coordenada_x = 300
        self.coordenada_y = 300
        self.thetax = 105/180*pi
        self.thetaz = -135/180*pi
        
    def rotar_display(self,x_spot,y_spot,z_spot,theta_spot,xl,yl,zl):
        p = [(0,0) for i in range(len(xl))]
        Ma = xyz_rotation_matrix(theta_spot[3],theta_spot[4],theta_spot[2]+theta_spot[5],False)
        Mb = xyz_rotation_matrix(theta_spot[0],theta_spot[1],0,False)
        M1 = xyz_rotation_matrix(self.thetax,0,self.thetaz,True)
    
        for i in range (0,len(xl)):    
            # Coordenadas absolutas de las líneas puntuales en los marcos x,y,z
            out0 = new_coordinates(Ma,xl[i],yl[i],zl[i],x_spot,y_spot,z_spot)
            out = new_coordinates(Mb,out0[0],out0[1],out0[2],0,0,0)
            
            #Coordenadas para visualizar en pantalla
            disp = new_coordinates(M1,out[0],out[1],out[2],0,0,0)
            
            puntos_2d = matriz_2d() @ np.vstack(disp)
            x = puntos_2d[0] * self.escala  + self.coordenada_x # Obtenemos la coordenada x
            y = puntos_2d[1] * self.escala  + self.coordenada_y # Obtenemos la coordenada y
            p[i] = (int(x),int(y))
        return p

    def draw_floor(self, pos):
        """Dibuja el suelo y la cuadrícula."""
        line = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                              [pos[12][0], pos[12][1], 0, 0, 0, 0],
                              coor.xFloor, coor.yFloor, coor.zFloor)
        pygame.draw.polygon(self.screen, color.GREY, line, 0)

        for i in range(11):
            grid_line_x = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [pos[12][0], pos[12][1], 0, 0, 0, 0],
                                         [-500 + i * 100, -500 + i * 100],
                                         [-500, 500], [0, 0])
            pygame.draw.lines(self.screen, color.DARK_GREY, False, grid_line_x, 1)

            grid_line_y = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
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
            line = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                  [0, 0, 0, 0, 0, 0], x, y, z)
            pygame.draw.lines(self.screen, color_axis, False, line, 2)

    def draw_radius_and_direction(self, pos, center_x, center_y, steering, walking_speed, walking_direction):
        """Dibuja el radio y la dirección."""
        if steering < 2000:
            line_radius = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [0, 0, 0, 0, 0, 0],
                                         [center_x, pos[13][0]], [center_y, pos[14][0]], [0, 0])
        else:
            center_x1 = pos[13][0] + (center_x - pos[13][0]) / steering * 2000
            center_y1 = pos[14][0] + (center_y - pos[14][0]) / steering * 2000
            line_radius = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                         [0, 0, 0, 0, 0, 0],
                                         [center_x1, pos[13][0]], [center_y1, pos[14][0]], [0, 0])

        xd = pos[13][0] + walking_speed * cos(pos[12][2] + walking_direction - pi / 2)
        yd = pos[14][0] + walking_speed * sin(pos[12][2] + walking_direction - pi / 2)
        line_direction = self.rotar_display(-pos[13][0], -pos[14][0], -pos[15][0],
                                        [0, 0, 0, 0, 0, 0],
                                        [xd, pos[13][0]], [yd, pos[14][0]], [0, 0])

        pygame.draw.lines(self.screen, color.CYAN, False, line_radius, 2)
        pygame.draw.lines(self.screen, color.GREEN, False, line_direction, 2)

    def draw_legs(self, pos, theta_spot, angles):
        """Dibuja las patas del robot."""
        legs = {
            "lf": {"x": torso.xlf, "y": torso.ylf, "angles": angles[0]},
            "rf": {"x": torso.xrf, "y": torso.yrf, "angles": angles[1]},
            "rr": {"x": torso.xrr, "y": torso.yrr, "angles": angles[2]},
            "lr": {"x": torso.xlr, "y": torso.ylr, "angles": angles[3]}
        }

        for leg, data in legs.items():
            fk = FK(data["angles"], 1 if leg in ["lf", "lr"] else -1)
            x_leg = [data["x"], data["x"] + fk[0], data["x"] + fk[1], data["x"] + fk[2], data["x"] + pos[0]]
            y_leg = [data["y"], data["y"] + fk[3], data["y"] + fk[4], data["y"] + fk[5], data["y"] + pos[1]]
            z_leg = [0, fk[6], fk[7], fk[8], pos[2]]

            line_leg = self.rotar_display(pos[13][1] - pos[13][0], pos[14][1] - pos[14][0], pos[15][1] - pos[15][0],
                                      theta_spot, x_leg, y_leg, z_leg)
            pygame.draw.lines(self.screen, color.RED, False, line_leg, 4)
            
    

    def animate(self, pos, t, angle, center_x, center_y, thetalf, thetarf, thetarr, thetalr, walking_speed, walking_direction, steering, stance):
        """Animación principal."""
        self.screen.fill(color.WHITE)

        # Dibujar elementos estáticos
        self.draw_floor(pos)
        self.draw_axes(pos)

        # Dibujar radio y dirección
        self.draw_radius_and_direction(pos, center_x, center_y, steering, walking_speed, walking_direction)

        # Dibujar patas
        angles = [thetalf, thetarf, thetarr, thetalr]
        self.draw_legs(pos, pos[12], angles)

        pygame.display.flip()