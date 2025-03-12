from src.config.mover_pie import seq, stepl
def calculate_compensation(stance, x_spot, y_spot, t, weight):
    """
    Calcula la compensación para mantener el equilibrio.
    
    Parámetros:
    - stance: Estado de apoyo de las patas.
    - x_spot, y_spot: Coordenadas actuales de las patas.
    - t1: Tiempo normalizado.
    - seq: Secuencia de tiempos para las patas.
    - weight: Peso para el cálculo del área de sustentación.
    
    Retorna:
    - x_abs_comp, y_abs_comp: Compensaciones absolutas.
    """
    stance_test = sum(stance)
    x_abs_area, y_abs_area = [0] * 4, [0] * 4
    t1 = t % 1
    

    for i in range(4):
        x_abs_area[i] = ((x_spot[3] + x_spot[5]) * weight + x_spot[4]) / (2 * weight + 1)
        y_abs_area[i] = ((y_spot[3] + y_spot[5]) * weight + y_spot[4]) / (2 * weight + 1)

    if stance_test == 4:
        istart = iend = 0
        tstart = int(t1 / 0.25) * 0.25
        tend = tstart + 0.25
        if tend == 1:
            tend = 0

        for i in range(4):
            if tstart == seq[i]:
                istart = i
            if tend == seq[i]:
                iend = i

        if t1 > (seq[istart] + stepl):
            x_abs_comp = x_abs_area[istart] + (x_abs_area[iend] - x_abs_area[istart]) * (t1 - tstart - stepl) / (0.25 - stepl)
            y_abs_comp = y_abs_area[istart] + (y_abs_area[iend] - y_abs_area[istart]) * (t1 - tstart - stepl) / (0.25 - stepl)
        else:
            x_abs_comp = x_abs_area[istart]
            y_abs_comp = y_abs_area[istart]
    else:
        for i in range(4):
            if not stance[i]:
                x_abs_comp = x_abs_area[i]
                y_abs_comp = y_abs_area[i]

    return x_abs_comp, y_abs_comp