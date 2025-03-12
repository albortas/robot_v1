from math import sin, cos, sqrt, atan2
import numpy as np
from src.config.puntos_torso import *
from src.config.dimensiones import L0

class RotacionPiePlanoXY:
    def __init__(self, radio, angulo):
        self.radio = radio
        self.angulo = angulo
    
    def centro_giro(self):
        """
        Calcula las coordenadas del centro de giro.
        
        Parámetros:
        - r: Radio de giro.
        - angulo: Ángulo de dirección.
        
        Retorna:
        - x, y: Coordenadas del centro de giro.
        """
        x = self.radio * cos(self.angulo)
        y = self.radio * sin(self.angulo)
        return x, y
    
    def posicion_nominal_pie(self):
        """
        Calcula las posiciones nominales de las patas.
        
        Parámetros:
        - L0: Dimension del ancho del hombro
        
        Retorna:
        - h, k: Posiciones nominales de los pie.
        """
        h = [xlf, xrf, xrr, xlr]
        k = [ylf + L0, yrf - L0, yrr - L0, ylr + L0]
        return h, k
    
    def radios_angulos_pies(self):
        """
        Calcula los radios y ángulos nominales de las patas.
                        
        Retorna:
        - radii: Radios de giro de las patas.
        - an: Ángulos nominales de las patas.
        """
        x, y = self.centro_giro()
        h, k = self.posicion_nominal_pie()
        radii = [0] * 4 #Genera una lista con 4 elementos ceros
        an = [0] * 4
        for i in range(4):
            radii[i] = sqrt((x - h[i]) ** 2 + (y - k[i]) ** 2)
            an[i] = atan2(k[i] - y, h[i] - x)
        return radii, an