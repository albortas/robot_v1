import pygame

pygame.init()


# Esta es una clase sencilla que nos ayudará a imprimir en la pantalla.
# No tiene nada que ver con los joysticks, solo con la salida de informacion

class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 25)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (0, 0, 0))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


def main():
    # Establezca el ancho y la altura de la pantalla (ancho, alto) y nombre la ventana.
    screen = pygame.display.set_mode((500, 500))
    pygame.display.set_caption("Ejemplo Joystick")

    # Se utiliza para administrar la velocidad con la que se actualiza la pantalla.
    clock = pygame.time.Clock()

    # Prepárese para imprimir.
    text_print = TextPrint()

    # Este diccionario se puede dejar como está, ya que Pygame generará un
    # evento pygame.JOYDEVICEADDED para cada joystick conectado
    # al inicio del programa.
    joysticks = {}

    done = False
    while not done:
        # Paso de procesamiento de eventos.
        # Posibles eventos de joystick: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Marca que hemos terminado para salir de este bucle.

            if event.type == pygame.JOYBUTTONDOWN:
                print("Boton Joystick presionado.")
                
            if event.type == pygame.JOYBUTTONUP:
                print("Boton del joystick liberado.")

            # Manejar la conexión en caliente
            if event.type == pygame.JOYDEVICEADDED:
                # Este evento se generará cuando el programa se inicie para cada
                # joystick, llenando la lista sin necesidad de crearlas manualmente.
                joy = pygame.joystick.Joystick(event.device_index)
                joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} conectado")

            if event.type == pygame.JOYDEVICEREMOVED:
                del joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} desconectado")

        # Paso de dibujo
        # Primero, limpia la pantalla y dejala en blanco. No uses otros comandos de dibujo
        # por encima de esto, o se borrarán con este comando.
        screen.fill((255, 255, 255))
        text_print.reset()

        # Obtener el recuento de joysticks.
        joystick_count = pygame.joystick.get_count()

        text_print.tprint(screen, f"Numero de joysticks: {joystick_count}")
        text_print.indent()

        # Para cada joystick:
        for joystick in joysticks.values():
            jid = joystick.get_instance_id()

            text_print.tprint(screen, f"Joystick {jid}")
            text_print.indent()

            # Obtenga el nombre del sistema operativo para el controlador/joystick.
            name = joystick.get_name()
            text_print.tprint(screen, f"Nombre del Joystick: {name}")

            guid = joystick.get_guid()
            text_print.tprint(screen, f"GUID: {guid}")

            power_level = joystick.get_power_level()
            text_print.tprint(screen, f"Nivel de potencia del joystick: {power_level}")

            # Generalmente los ejes se ejecutan en pares, arriba/abajo para uno, e izquierda/derecha para el otro
            # el otro. Los disparadores cuentan como ejes.
            axes = joystick.get_numaxes()
            text_print.tprint(screen, f"Numero de ejes: {axes}")
            text_print.indent()

            for i in range(axes):
                axis = joystick.get_axis(i)
                text_print.tprint(screen, f"Eje {i} valor: {axis:>6.3f}")
            text_print.unindent()

            buttons = joystick.get_numbuttons()
            text_print.tprint(screen, f"Numero de botones: {buttons}")
            text_print.indent()

            for i in range(buttons):
                button = joystick.get_button(i)
                text_print.tprint(screen, f"Boton {i:>2} valor: {button}")
            text_print.unindent()

            hats = joystick.get_numhats()
            text_print.tprint(screen, f"Numero de harts: {hats}")
            text_print.indent()

            # Posición del sombrero. Todo o nada para la dirección, no un flotador como
            # get_axis(). La posición es una tupla de valores int (x, y).
            for i in range(hats):
                hat = joystick.get_hat(i)
                text_print.tprint(screen, f"Hat {i} valor: {str(hat)}")
            text_print.unindent()

            text_print.unindent()

        # Continúa y actualiza la pantalla con lo que hemos dibujado.
        pygame.display.flip()

        # Límite de 30 cuadros por segundo.
        clock.tick(30)


if __name__ == "__main__":
    main()
    # Si olvida esta línea, el programa se 'colgará'
    # al salir si se ejecuta desde IDLE.
    pygame.quit()