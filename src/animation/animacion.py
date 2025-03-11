from math import pi, sin, cos
import pygame
import src.config.puntos_torso as torso
import src.config.colores as color
import src.config.coordenadas as coor
from src.utils.cinematica import FK
from src.animation.display import display_rotate


class SpotAnime:
    """
    Clase para animar un robot cuadrúpedo en Pygame.
    
    Gestiona la representación visual de:
    - Entorno (suelo y cuadrícula)
    - Sistema de referencia (ejes X, Y, Z)
    - Trayectorias de movimiento
    - Cinemática de las patas
    - Centro de gravedad
    - Área de sustentación
    - Información de depuración
    """
    
    def animate(self, pos,t, thetax, thetaz, angle, center_x, center_y, 
                thetalf, thetarf, thetarr, thetalr, walking_speed, 
                walking_direction, steering, stance):
        """
        Método principal que coordina todos los elementos de la animación
        
        Args:
            pos (list): Posiciones articulares y coordenadas del robot
            thetax (float): Ángulo de rotación en eje X (cabeceo)
            thetaz (float): Ángulo de rotación en eje Z (guinada)
            angle (list): Ángulos de articulación para visualización
            center_x (float): Coordenada X del centro de giro
            center_y (float): Coordenada Y del centro de giro
            thetalf (list): Ángulos de la pata delantera izquierda
            thetarf (list): Ángulos de la pata delantera derecha
            thetarr (list): Ángulos de la pata trasera derecha
            thetalr (list): Ángulos de la pata trasera izquierda
            walking_speed (float): Velocidad de desplazamiento
            walking_direction (float): Dirección de movimiento (radianes)
            steering (float): Radio de giro
            stance (list): Estados de contacto de las patas (True=apoyada)
        """
        
        # Extraer coordenadas del robot
        theta_spot = pos[12]  # Orientación del robot
        x_spot = pos[13]      # Coordenadas X de componentes
        y_spot = pos[14]      # Coordenadas Y de componentes
        z_spot = pos[15]      # Coordenadas Z de componentes
        
        # Centro de gravedad absoluto y desplazamiento
        CGabs = [x_spot[7], y_spot[7], z_spot[7]]  # Posición CG
        dCG = [x_spot[8], y_spot[8], z_spot[8]]    # Desplazamiento CG

        self.screen.fill(color.WHITE)
        
        # Dibujar componentes
        self.draw_floor_grid(theta_spot, thetax, thetaz, x_spot, y_spot, z_spot)
        self.draw_xyz_frame(thetax, thetaz, x_spot, y_spot, z_spot)
        self.draw_radius_and_direction(center_x, center_y, steering, x_spot, y_spot, 
                                      walking_speed, walking_direction, theta_spot, thetax, thetaz)
        self.draw_legs(pos, theta_spot, thetax, thetaz, x_spot, y_spot, z_spot, 
                      thetalf, thetarf, thetarr, thetalr)
        self.draw_body(theta_spot, thetax, thetaz, x_spot, y_spot, z_spot)
        self.draw_sustentation_area(stance, pos, theta_spot, thetax, thetaz, x_spot, y_spot, z_spot)
        self.draw_center_of_gravity(CGabs, dCG, x_spot, y_spot, z_spot, thetax, thetaz)
        self.draw_debug_info(angle)
        
        pygame.display.flip()

    def draw_floor_grid(self, theta_spot, thetax, thetaz, x_spot, y_spot, z_spot):
        """Dibuja el suelo y la cuadrícula de referencia"""
        # Suelo principal
        line = display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0],
                             [theta_spot[0], theta_spot[1], 0, 0, 0, 0],
                             thetax, thetaz, coor.xFloor, coor.yFloor, coor.zFloor)
        pygame.draw.polygon(self.screen, color.GREY, line, 0)
        
        # Cuadrícula de 10x10 (11 líneas)
        for i in range(11):
            # Líneas verticales
            line = display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0],
                                [theta_spot[0], theta_spot[1], 0, 0, 0, 0],
                                thetax, thetaz, [-500+i*100, -500+i*100],
                                [-500, 500], [0, 0])
            pygame.draw.lines(self.screen, color.DARK_GREY, False, line, 1)
            
            # Líneas horizontales
            line = display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0],
                                [theta_spot[0], theta_spot[1], 0, 0, 0, 0],
                                thetax, thetaz, [-500, 500],
                                [-500+i*100, -500+i*100], [0, 0])
            pygame.draw.lines(self.screen, color.DARK_GREY, False, line, 1)

    def draw_xyz_frame(self, thetax, thetaz, x_spot, y_spot, z_spot):
        """Dibuja los ejes de coordenadas X, Y, Z"""
        for axis, col in zip(['X', 'Y', 'Z'], [color.RED, color.GREEN, color.BLUE]):
            line = display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0],
                                [0, 0, 0, 0, 0, 0], thetax, thetaz,
                                getattr(coor, f'x{axis}'), getattr(coor, f'y{axis}'),
                                getattr(coor, f'z{axis}'))
            pygame.draw.lines(self.screen, col, False, line, 2)

    def draw_radius_and_direction(self, center_x, center_y, steering, x_spot, y_spot, 
                                walking_speed, walking_direction, theta_spot, thetax, thetaz):
        """Dibuja el radio de giro y dirección de movimiento"""
        # Radio de giro
        if steering < 2000:
            lineR = display_rotate(self, -x_spot[0], -y_spot[0], 0,
                                  [0, 0, 0, 0, 0, 0], thetax, thetaz,
                                  [center_x, x_spot[0]], [center_y, y_spot[0]], [0, 0])
            center_display = True
        else:
            # Ajuste para radios grandes
            center_x1 = x_spot[0] + (center_x - x_spot[0])/steering*2000
            center_y1 = y_spot[0] + (center_y - y_spot[0])/steering*2000
            lineR = display_rotate(self, -x_spot[0], -y_spot[0], 0,
                                  [0, 0, 0, 0, 0, 0], thetax, thetaz,
                                  [center_x1, x_spot[0]], [center_y1, y_spot[0]], [0, 0])
            center_display = False
        pygame.draw.lines(self.screen, color.CYAN, False, lineR, 2)
        
        # Dirección de movimiento
        xd = x_spot[0] + walking_speed * cos(theta_spot[2] + walking_direction - pi/2)
        yd = y_spot[0] + walking_speed * sin(theta_spot[2] + walking_direction - pi/2)
        lineD = display_rotate(self, -x_spot[0], -y_spot[0], 0,
                              [0, 0, 0, 0, 0, 0], thetax, thetaz,
                              [xd, x_spot[0]], [yd, y_spot[0]], [0, 0])
        pygame.draw.lines(self.screen, color.GREEN, False, lineD, 2)
        
        # Marcadores de centro
        if center_display:
            pygame.draw.circle(self.screen, color.BLACK, lineR[0], 5)
        pygame.draw.circle(self.screen, color.DARK_CYAN, lineR[1], 5)

    def draw_legs(self, pos, theta_spot, thetax, thetaz, x_spot, y_spot, z_spot, 
                thetalf, thetarf, thetarr, thetalr):
        """Dibuja las cuatro patas del robot usando cinemática directa"""
        legs = [
            (thetalf, 1, torso.xlf, torso.ylf, pos[0], pos[1], pos[2]),
            (thetarf, -1, torso.xrf, torso.yrf, pos[3], pos[4], pos[5]),
            (thetarr, -1, torso.xrr, torso.yrr, pos[6], pos[7], pos[8]),
            (thetalr, 1, torso.xlr, torso.ylr, pos[9], pos[10], pos[11])
        ]
        
        for theta, side, x_torso, y_torso, x_pos, y_pos, z_pos in legs:
            # Calcular cinemática directa
            leg_points = FK(theta, side)
            
            # Coordenadas de los segmentos de la pata
            x_leg = [x_torso, x_torso+leg_points[0], x_torso+leg_points[1], 
                    x_torso+leg_points[2], x_torso+x_pos]
            y_leg = [y_torso, y_torso+leg_points[3], y_torso+leg_points[4], 
                    y_torso+leg_points[5], y_torso+y_pos]
            z_leg = [0, leg_points[6], leg_points[7], leg_points[8], z_pos]
            
            # Transformar coordenadas 3D a 2D
            line = display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], 
                                z_spot[1]-z_spot[0], theta_spot, thetax, thetaz,
                                x_leg, y_leg, z_leg)
            pygame.draw.lines(self.screen, color.RED, False, line, 4)

    def draw_body(self, theta_spot, thetax, thetaz, x_spot, y_spot, z_spot):
        """Dibuja la estructura principal del cuerpo del robot"""
        body_points = [
            (torso.xlf, torso.ylf, torso.zlf),
            (torso.xrf, torso.yrf, torso.zrf),
            (torso.xrr, torso.yrr, torso.zrr),
            (torso.xlr, torso.ylr, torso.zlr),
            (torso.xlf, torso.ylf, torso.zlf)  # Cerrar polígono
        ]
        
        # Extraer coordenadas
        x_body = [p[0] for p in body_points]
        y_body = [p[1] for p in body_points]
        z_body = [p[2] for p in body_points]
        
        # Transformar y dibujar
        lineb = display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], 
                              z_spot[1]-z_spot[0], theta_spot, thetax, thetaz,
                              x_body, y_body, z_body)
        pygame.draw.lines(self.screen, color.BLUE, False, lineb, 10)

    def draw_sustentation_area(self, stance, pos, theta_spot, thetax, thetaz, x_spot, y_spot, z_spot):
        """Dibuja el área de sustentación (polígono de apoyo)"""
        linesus = []
        for i in range(4):
            if stance[i]:
                # Coordenadas de las patas en contacto
                x = [torso.xlf, torso.xrf, torso.xrr, torso.xlr][i]
                y = [torso.ylf, torso.yrf, torso.yrr, torso.ylr][i]
                z = [pos[2], pos[5], pos[8], pos[11]][i]
                
                # Transformar coordenadas
                point = display_rotate(self, x_spot[1]-x_spot[0], y_spot[1]-y_spot[0], 
                                      z_spot[1]-z_spot[0], theta_spot, thetax, thetaz,
                                      [x], [y], [z])
                linesus.append(point[0])
        
        if linesus:
            pygame.draw.polygon(self.screen, color.VIOLET, linesus, 0)
            pygame.draw.lines(self.screen, color.BLACK, True, linesus, 1)

    def draw_center_of_gravity(self, CGabs, dCG, x_spot, y_spot, z_spot, thetax, thetaz):
        """Dibuja el centro de gravedad y su desplazamiento"""
        # Línea vertical del CG
        lineCG = display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0],
                               [0, 0, 0, 0, 0, 0], thetax, thetaz,
                               [CGabs[0], CGabs[0]], [CGabs[1], CGabs[1]],
                               [0, CGabs[2]])
        pygame.draw.lines(self.screen, color.BLACK, False, lineCG, 1)
        pygame.draw.circle(self.screen, color.DARK_GREY, lineCG[1], 10)
        
        # Desplazamiento del CG
        linedCG = display_rotate(self, -x_spot[0], -y_spot[0], -z_spot[0],
                                [0, 0, 0, 0, 0, 0], thetax, thetaz,
                                [CGabs[0], dCG[0]], [CGabs[1], dCG[1]], [0, 0])
        pygame.draw.lines(self.screen, color.BLACK, False, linedCG, 1)
        
        # Color según estabilidad
        color_cg = color.GREEN if dCG[2] else color.RED
        pygame.draw.circle(self.screen, color_cg, lineCG[0], 3)

    def draw_debug_info(self, angle):
        """Dibuja información de depuración (ángulos)"""
        # Ángulo de cabeceo (X)
        pygame.draw.lines(self.screen, color.BLACK, False,
                         [[angle[0]/pi*180/45*300+300, 0], 
                         [angle[0]/pi*180/45*300+300, 50]], 5)
                         
        # Ángulo de guinada (Z)
        pygame.draw.lines(self.screen, color.BLACK, False,
                         [[0, angle[1]/pi*180/45*300+300], 
                         [50, angle[1]/pi*180/45*300+300]], 5)