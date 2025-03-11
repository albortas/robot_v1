import pygame

import src.animation.animacion as animacion
anime = animacion.SpotAnime()

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True
dt = 0

player_pos = pygame.Vector2(screen.get_width() / 2, screen.get_height() / 2)

theta_spot, thetax, thetaz, x_spot, y_spot, z_spot = 0, 0, 0, 0, 0, 0

def teclado(dt):
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        player_pos.y -= 300 * dt
    if keys[pygame.K_s]:
        player_pos.y += 300 * dt
    if keys[pygame.K_a]:
        player_pos.x -= 300 * dt
    if keys[pygame.K_d]:
        player_pos.x += 300 * dt
    
    return player_pos.x, player_pos.y

while running:
    # poll for events
    # El evento pygame.QUIT significa que el usuario hizo clic en X para cerrar su ventana
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Llena la pantalla con un color para borrar todo lo del último fotograma.
    screen.fill("white")

    #pygame.draw.circle(screen, "red", player_pos, 40)

    #player_pos.x, player_pos.y = teclado(dt)
    
    anime.draw_floor_grid(theta_spot, thetax, thetaz, x_spot, y_spot, z_spot)

    # voltea() la pantalla para poner tu trabajo en la pantalla
    pygame.display.flip()

    # limita los FPS a 60
    # dt es el tiempo delta en segundos desde el último fotograma,
    # utilizado para física independiente de la velocidad de fotogramas.
    dt = clock.tick(60) / 1000

pygame.quit()

