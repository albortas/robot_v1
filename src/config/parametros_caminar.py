from math import pi, cos, sin
import numpy as np

from src.config.puntos_torso import *

""" Walking parameters """
b_height = 200
h_amp = 100# horizontal step length
v_amp = 40 #vertical step length
track = 58.09
x_offset = 0 #body offset in x direction 
ra_longi = 30# body distance 
ra_lat = 30#20
steering =200 #Initial steering radius (arbitrary)
walking_direction = 90/180*pi #Initial steering angle (arbitrary)
stepl = 0.125 #duration of leg lifting typically between 0.1 and 0.2
seq = [0, 0.5, 0.25, 0.75] # secuencia de tiempos para los pies

Angle = [0, 0]
center_x = steering*cos(walking_direction) #steering center x relative to body center 
center_y = steering*sin(walking_direction) #steering center y relative to body center
cw =1

stance = [True, True, True, True] # True = foot on the ground, False = Foot lifted
theta_spot = [0,0,0,0,0,0]

""" XBOX One controller settings """
""" use Essai_Joystick_01.py utility to find out the right parameters """

nj = 6 # Number of joysticks
nb = 10 # number of buttons

but_walk = 7
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

joypos =np.zeros(6) #xbox one controller has 6 analog inputs
joybut = np.zeros(10) #xbox one controller has 10 buttons

continuer = True
t = 0 #Initializing timing/clock
tstart = 1 #End of start sequence
tstop = 1000 #Start of stop sequence by default
tstep = 0.01 #Timing/clock step
tstep1 = tstep

distance =[] #distance to support polygon edge
balance =[] #balance status (True or False)
timing = [] #timing to plot distance


Free = True #Spot is ready to receive new command
sitting = False 
walking = False #walking sequence activation
lying = False 
twisting = False
pawing = False
shifting = False
peeing = False
stop = False # walking stop sequence activation
lock = False # locking key/button stroke as a "rebound filter"
lockmouse = False
mouseclick = False

walking_speed = 0
walking_direction = 0
steeering = 1e6
module = 0
joype = -1 # Initial joysick value for peeing
joypar = -1 # Initial joysick value for pawing right
joypal = -1 # Initial joysick value for pawing left

Tcomp = 0.02

x_spot = [0, x_offset, xlf, xrf, xrr, xlr,0,0,0]
y_spot = [0,0, ylf+track, yrf-track, yrr-track, ylr+track,0,0,0]
z_spot = [0,b_height,0,0,0,0,0,0,0]

