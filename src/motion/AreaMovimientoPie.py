from math import sin, cos, sqrt, atan2
import numpy as np
from src.config.puntos_torso import *
from src.config.dimensiones import L0

class AreaMovimientoPie:
    def __init__(self,dir_radio, dir_angulo):
        self.dir_radio = dir_radio
        self.dir_angulo = dir_angulo
    
    
    """Coordenadas del centro de dirección en el marco puntual
    """
    def coordenas_circulo(self):
        xc = self.dir_radio* cos(self.dir_angulo)
        yc = self.dir_radio* sin(self.dir_angulo)
        return xc, yc

    """Posicion Nominal Pie
    """
    def posicion_nominal_pie(self):
        xn = [xlf, xrf, xrr, xlr]
        yn = [ylf + L0, yrf - L0, yrr-L0, ylr+L0]
        return xn, yn

    """Direccion del radio y el angulo nominal al pie desde el centro del robot
    """
    def radio_angulo(self):
        xc, yc = self.coordenas_circulo()
        xn, yn = self.posicion_nominal_pie()
        radii = np.zeros(4)
        an = np.zeros(4)
        for i in range (0,4): 
            """ Direccion del Radio"""
            """ Ecuacion de la circunferencia que se aplica en el plano xy""" 
            radii[i] = sqrt((xc-xn[i])**2+(yc-yn[i])**2)
            """ Angulo Nominal al pie"""  
            an[i] = atan2(yn[i]-yc,xn[i]-xc) 
        return radii, an      