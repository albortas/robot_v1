from math import pi
from src.config.mover_pie import seq, stepl

def angulo_rotacion(step_phase, t):
    """
    Calcula el ángulo de rotación para la fase actual.
    
    Parámetros:
    - step_phase: Fase de caminata ('start', 'walk', 'stop').
    - t1: Tiempo normalizado.
    - seq: Secuencia de tiempos para los pies.
    - stepl: Duración del paso.
    
    Retorna:
    - alpha: Ángulos de rotación de los pies.
    - alphav: Ángulos verticales de los pies.
    - stance: Estado de apoyo de los pies.
    """
    alpha, alphav = [0]*4, [0]*4
    stance = [True] * 4
    t1 = t%1

    for i in range(4):
        if t1 <= seq[i]:
            stance[i] = True  # Pata en el suelo
        else:
            if t1 < (seq[i] + stepl):
                stance[i] = False  # Pata levantada
                alphav[i] = -pi / 2 + 2 * pi / stepl * (t1 - seq[i])
                t2 = seq[i] + stepl

                if step_phase == 'start':
                    alpha[i] = -seq[i] / (1 - stepl) / 2 + (t2 - seq[i]) / stepl / (1 - stepl) * seq[i]
                elif step_phase == 'stop':
                    alpha[i] = -1 / 2 + seq[i] / (1 - stepl) / 2 + (t2 - seq[i]) / stepl * (1 - seq[i] / (1 - stepl))
                else:
                    alpha[i] = -1 / 2 + ((t2 - seq[i]) / stepl)
            else:
                stance[i] = True  # Pata en el suelo

    return alpha, alphav, stance

if __name__ == '__main__':
    print("Ejecucion para los angulos de rotacion actual")
    st_a, st_av, st_s = angulo_rotacion('start',0.7)
    sp_a, sp_av, sp_s = angulo_rotacion('stop',0.756)
    w_a, w_av, w_s = angulo_rotacion('walk',0.756)
    print(st_a, st_av, st_s)
    print(sp_a, sp_av, sp_s)
    print(w_a, w_av, w_s)
    
    