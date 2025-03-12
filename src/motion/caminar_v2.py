import numpy as np
from math import pi, sin, cos, atan2, sqrt
from src.utils.operador import xyz_rotation_matrix, new_coordinates
from src.config.puntos_torso import *
from src.config.mover_pie import seq

# Ángulo óptimo cuando la pata está completamente levantada
phase = pi / 8

def calculate_body_frame_position(comp, CG, x_offset, theta_spot_updated, height, foot_center, kcomp):
    """
    Calcula la nueva posición del centro del cuerpo.
    
    Parámetros:
    - comp: Compensación calculada.
    - CG: Centro de gravedad.
    - x_offset: Desplazamiento horizontal.
    - theta_spot_updated: Orientación actualizada del cuerpo.
    - height: Altura del cuerpo.
    - foot_center: Centro nominal de las patas.
    - kcomp: Factor de compensación.
    
    Retorna:
    - x_framecenter_comp, y_framecenter_comp, z_framecenter_comp: Nueva posición del cuerpo.
    """
    Ms_comp = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)
    compt = new_coordinates(Ms_comp, (comp[0] - CG[0]) * kcomp + x_offset, (comp[1] - CG[1]) * kcomp, 0, 0, 0, 0)

    x_framecenter_comp = foot_center[0] + compt[0]
    y_framecenter_comp = foot_center[1] + compt[1]
    z_framecenter_comp = height

    return x_framecenter_comp, y_framecenter_comp, z_framecenter_comp

def update_leg_positions(
    stance, alpha, alphav, radii, an, v_amp, v_amp_t, comp, CG, x_offset, x_frame, y_frame, z_frame,
    x_framecorner, y_framecorner, z_framecorner, Msi_comp, Msi_updated, Ms_updated, theta_spot_updated, seq, stepl, tstep
):
    """
    Actualiza las posiciones de las patas.
    
    Parámetros:
    - stance: Estado de apoyo de las patas.
    - alpha, alphav: Ángulos de rotación y verticales.
    - radii, an: Radios y ángulos nominales.
    - v_amp, v_amp_t: Amplitudes verticales.
    - comp: Compensación calculada.
    - CG: Centro de gravedad.
    - x_offset: Desplazamiento horizontal.
    - x_frame, y_frame, z_frame: Esquinas del cuerpo.
    - x_framecorner, y_framecorner, z_framecorner: Esquinas actualizadas.
    - Msi_comp, Msi_updated, Ms_updated: Matrices de rotación.
    - theta_spot_updated: Orientación actualizada.
    - seq, stepl, tstep: Parámetros de tiempo.
    
    Retorna:
    - xleg, yleg, zleg: Posiciones relativas de las patas.
    - xabs, yabs, zabs: Posiciones absolutas de las patas.
    """
    xleg = np.zeros(4)
    yleg = np.zeros(4)
    zleg = np.zeros(4)
    xabs = np.zeros(4)
    yabs = np.zeros(4)
    zabs = np.zeros(4)

    for i in range(4):
        if not stance[i]:
            alphah = an[i] + mangle * alpha[i] * cw
            xleg_target = xc + radii[i] * cos(alphah) - (comp[0] - CG[0]) * kcomp - x_offset - x_frame[i]
            yleg_target = yc + radii[i] * sin(alphah) - (comp[1] - CG[1]) * kcomp - y_frame[i]

            leg_current = new_coordinates(Msi_comp, x_spot[i + 2] - x_framecorner[i], y_spot[i + 2] - y_framecorner[i], -z_framecorner[i], 0, 0, 0)

            if (seq[i] + stepl - t1) > tstep:
                xint = leg_current[0] + (xleg_target - leg_current[0]) * tstep / (seq[i] + stepl - t1)
                yint = leg_current[1] + (yleg_target - leg_current[1]) * tstep / (seq[i] + stepl - t1)
            else:
                xint, yint = xleg_target, yleg_target

            zint = leg_current[2] + v_amp_t * (1 + sin(alphav[i])) / 2

            legs = new_coordinates(Msi_body, xint, yint, zint, 0, 0, 0)
            xleg[i], yleg[i], zleg[i] = legs

            foot_abs = new_coordinates(Ms_updated, xleg[i], yleg[i], zleg[i], x_framecorner[i], y_framecorner[i], z_framecorner[i])
            xabs[i], yabs[i], zabs[i] = foot_abs
        else:
            xabs[i], yabs[i], zabs[i] = x_spot[i + 2], y_spot[i + 2], 0

            leg = new_coordinates(Msi_updated, xabs[i] - x_framecorner[i], yabs[i] - y_framecorner[i], zabs[i] - z_framecorner[i], 0, 0, 0)
            xleg[i], yleg[i], zleg[i] = leg

    return xleg, yleg, zleg, xabs, yabs, zabs

def start_walk_stop(
    track, x_offset, steering_radius, steering_angle, cw, h_amp, v_amp, height,
    stepl, t, tstep, theta_spot, x_spot, y_spot, z_spot, step_phase
):
    """
    Función principal para generar las posiciones de caminata del robot.
    
    Parámetros:
    - track: Distancia entre las patas izquierda y derecha.
    - x_offset: Desplazamiento horizontal del cuerpo del robot.
    - steering_radius: Radio de giro del robot.
    - steering_angle: Ángulo de dirección del robot.
    - cw: Dirección de giro (1 para horario, -1 para antihorario).
    - h_amp: Amplitud horizontal del movimiento.
    - v_amp: Amplitud vertical del movimiento.
    - height: Altura del cuerpo del robot.
    - stepl: Duración del paso en segundos.
    - t: Tiempo actual en segundos.
    - tstep: Incremento de tiempo por iteración.
    - theta_spot: Orientación inicial del cuerpo del robot.
    - x_spot, y_spot, z_spot: Coordenadas iniciales de las patas y el cuerpo.
    - step_phase: Fase de caminata ('start', 'walk', 'stop').
    
    Retorna:
    - pos: Lista con las posiciones actualizadas de las patas y el cuerpo.
    """
    # Inicialización de variables
    theta_spot_updated = theta_spot.copy()
    CG = [x_spot[6], y_spot[6], z_spot[6]]
    xc, yc = calculate_steering_center(steering_radius, steering_angle)
    xn, yn = calculate_nominal_foot_positions(track)
    radii, an = calculate_radii_and_angles(xn, yn, xc, yc)
    maxr = max(radii)
    mangle = h_amp / maxr

    # Cálculo del ángulo de rotación
    dtheta = mangle / (1 - stepl) * tstep * cw
    theta_spot_updated[2] += dtheta

    # Matrices de rotación
    Ms_updated = xyz_rotation_matrix(theta_spot_updated[3], theta_spot_updated[4], theta_spot_updated[2] + theta_spot_updated[5], False)
    Msi_updated = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4], -(theta_spot_updated[2] + theta_spot_updated[5]), True)
    dMs = xyz_rotation_matrix(0, 0, dtheta, False)

    # Posición absoluta del centro nominal de las patas
    foot_center = new_coordinates(dMs, x_spot[0] - xc, y_spot[0] - yc, 0, xc, yc, 0)

    # Cálculo del estado de las patas
    t1 = t % 1
    alpha, alphav, stance = calculate_rotation_angle(step_phase, t1, seq, stepl, mangle, cw)

    # Cálculo de compensación
    x_abs_comp, y_abs_comp = calculate_compensation(stance, x_spot, y_spot, theta_spot_updated, t1, seq, weight=1.2)

    # Compensación en el marco del cuerpo
    Msi_comp = xyz_rotation_matrix(0, 0, -theta_spot_updated[2], True)
    comp = new_coordinates(Msi_comp, x_abs_comp - x_spot[0], y_abs_comp - y_spot[0], 0, 0, 0, 0)

    # Compensación con ajuste de amplitud vertical
    ts = 0.25
    kcomp = 1
    v_amp_t = v_amp
    if step_phase == 'start' and t1 < ts:
        kcomp = t1 / ts
        v_amp_t = 0
    elif step_phase == 'stop' and t1 > (1 - ts):
        kcomp = (1 - t1) / ts
        v_amp_t = 0

    # Nueva posición del centro del cuerpo
    x_framecenter_comp, y_framecenter_comp, z_framecenter_comp = calculate_body_frame_position(
        comp, CG, x_offset, theta_spot_updated, height, foot_center, kcomp
    )

    # Nuevas posiciones de las esquinas del cuerpo
    x_frame = [xlf, xrf, xrr, xlr]
    y_frame = [ylf, yrf, yrr, ylr]
    z_frame = [0, 0, 0, 0]
    x_framecorner, y_framecorner, z_framecorner = np.zeros(4), np.zeros(4), np.zeros(4)
    for i in range(4):
        frame_corner = new_coordinates(Ms_updated, x_frame[i], y_frame[i], z_frame[i], x_framecenter_comp, y_framecenter_comp, z_framecenter_comp)
        x_framecorner[i], y_framecorner[i], z_framecorner[i] = frame_corner

    # Actualización de las posiciones de las patas
    xleg, yleg, zleg, xabs, yabs, zabs = update_leg_positions(
        stance, alpha, alphav, radii, an, v_amp, v_amp_t, comp, CG, x_offset, x_frame, y_frame, z_frame,
        x_framecorner, y_framecorner, z_framecorner, Msi_comp, Msi_updated, Ms_updated, theta_spot_updated, seq, stepl, tstep
    )

    # Actualización de las coordenadas del cuerpo y las patas
    x_spot_updated = [foot_center[0], x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3], x_spot[6], x_spot[7], x_spot[8]]
    y_spot_updated = [foot_center[1], y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3], y_spot[6], y_spot[7], y_spot[8]]
    z_spot_updated = [foot_center[2], z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3], z_spot[6], z_spot[7], z_spot[8]]

    # Retorno de las posiciones actualizadas
    pos = [
        xleg[0], yleg[0], zleg[0], xleg[1], yleg[1], zleg[1], xleg[2], yleg[2], zleg[2], xleg[3], yleg[3], zleg[3],
        theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated
    ]
    return pos