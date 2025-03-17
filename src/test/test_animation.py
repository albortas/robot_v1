import pygame
from src.animation.animacion import SpotAnime

# Inicializar Pygame
pygame.init()
screen = pygame.display.set_mode((600, 600))

# Crear instancia de SpotAnime
spot_anime = SpotAnime()
spot_anime.screen = screen

# Parámetros simulados
pos = [0] * 16
t = 0
thetax = 0
thetaz = 0
angle = [0, 0]
center_x = 0
center_y = 0
thetalf = [0, 0, 0]
thetarf = [0, 0, 0]
thetarr = [0, 0, 0]
thetalr = [0, 0, 0]
walking_speed = 0
walking_direction = 0
steering = 0
stance = [True, True, True, True]

# Bucle principal
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Animar el robot
    spot_anime.dibujar_cuadricula_piso(theta_spot, thetax, thetaz, x_spot, y_spot, z_spot)

pygame.quit()