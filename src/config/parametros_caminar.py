from math import pi, cos, sin
import numpy as np

from src.config.puntos_torso import *

""" Parámetros de marcha """
b_height = 200
h_amp = 100 # Longitud del paso horizontal
v_amp = 40 # Longitud del paso vertical
track = 58.09
x_offset = 0 # Desplazamiento del cuerpo en la dirección x
ra_longi = 30 # Distancia corporal 
ra_lat = 30 #20
steering =200 # Radio de dirección inicial (arbitrario)
walking_direction = 90/180*pi # Ángulo de dirección inicial (arbitrario)
stepl = 0.125 # La duración del levantamiento de la pierna suele estar entre 0,1 y 0,2
seq = [0, 0.5, 0.25, 0.75] # Secuencia de tiempos para los pies

Angle = [0, 0]
center_x = steering*cos(walking_direction) # Centro de dirección x con respecto al centro de la carrocería 
center_y = steering*sin(walking_direction) # Centro de dirección y con respecto al centro de la carrocería
cw =1

stance = [True, True, True, True] # Verdadero = pie en el suelo, Falso = pie levantado
theta_spot = [0,0,0,0,0,0]

""" Configuración del mando de PS4 """

nj = 6 # Número de joysticks
nb = 13 # Número de botones

but_walk = 12
but_sit = 2
but_lie = 3
but_twist = 1
but_pee = 0
but_move = 4
but_anim = 5

pos_frontrear = 4
pos_leftright = 3
pos_turn = 0    
pos_rightpaw = 5
pos_leftpaw = 2

joypos =np.zeros(nj) #xbox one controller has 6 analog inputs
joybut = np.zeros(nb) #xbox one controller has 10 buttons

continuer = True
t = 0 # Inicializando temporización/reloj
tstart = 1 # Fin de la secuencia de inicio
tstop = 1000 # Inicio de secuencia de parada por defecto
tstep = 0.01 # Paso de tiempo/reloj
tstep1 = tstep

distance =[] # Distancia al borde del polígono de soporte
balance =[] # Estado de saldo (Verdadero o Falso)
timing = [] # Tiempo para trazar la distancia


libre = True # Robot está listo para recibir nuevo comando
sitting = False 
caminando = False # Activación de la secuencia de caminata
lying = False 
twisting = False
pawing = False
shifting = False
peeing = False
stop = False # Activación de la secuencia de parada de marcha
cerrar = False # Bloqueo del recorrido de la tecla/botón como "filtro de rebote"
lockmouse = False
mouseclick = False

caminando_speed = 0
caminando_direction = 0
steeering = 1e6
module = 0
joype = -1 # Valor inicial de la alegría al orinar
joypar = -1 # Valor inicial del joystick para tocar la derecha
joypal = -1 # Valor inicial del joystick para mover la pata hacia la izquierda

Tcomp = 0.02


