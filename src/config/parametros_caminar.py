from math import pi, cos, sin
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

"""theta_spot = [x angle ground, y angle ground, z angle body in space, x angle body, y angle body, z angle body] """
theta_spot = [0,0,0,0,0,0]
theta_x = pi/12
theta_z = - pi * (135/180)
