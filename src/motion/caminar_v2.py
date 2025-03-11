from math import pi, sin, cos, atan2, sqrt
import numpy as np

from src.utils.operador import xyz_rotation_matrix,new_coordinates
from src.config.puntos_torso import *
from src.config.mover_pie import seq
stepl = 0.2  # Longitud del paso

def start_walk_stop( track, x_offset, steering_radius, steering_angle, cw, h_amp, v_amp, height, stepl, t,
                    tstep, theta_spot, x_spot, y_spot, z_spot, step_phase):
    """
    Controla el ciclo de marcha del robot, calculando las posiciones de las patas y ajustando los parámetros de movimiento.
    """
    # Paso 1: Calcular el centro de dirección
    xc, yc = calculate_steering_center(steering_radius, steering_angle)

    # Paso 2: Transformar el centro de dirección al marco del punto
    Ms = xyz_rotation_matrix(0, 0, theta_spot[2], False)
    xs, ys = transform_steering_center(Ms, xc, yc, x_spot, y_spot, z_spot)

    # Paso 3: Definir posiciones de las patas
    xn, yn = define_foot_positions(track)

    # Paso 4: Calcular radios y ángulos
    radii, an = calculate_radii_and_angles(xc, yc, xn, yn)

    # Paso 5: Calcular ángulo de movimiento y rotación
    mangle = calculate_movement_angle(radii, h_amp)
    dtheta = calculate_rotation_angle(mangle, stepl, tstep, cw, step_phase, theta_spot)
    theta_spot_updated = theta_spot.copy()
    theta_spot_updated[2] += dtheta

    # Paso 6: Actualizar matrices de rotación
    Ms_updated, Msi_updated, dMs = update_rotation_matrices(theta_spot_updated, dtheta)
    foot_center = calculate_foot_center(dMs, xs, ys, x_spot, y_spot)

    # Paso 7: Inicializar variables de apoyo
    stance_test, stance, t1 = initialize_stance_and_time(t, np.zeros(4), np.zeros(4), step_phase)

    # Paso 8: Calcular compensación y posiciones finales
    x_abs_area, y_abs_area = calculate_support_area(x_spot, y_spot)
    x_abs_comp, y_abs_comp = calculate_transition_area(t1, stance_test, stance, x_abs_area, y_abs_area, stepl)
    comp, compt, v_amp_t, Msi_comp = calculate_compensation(
        x_abs_comp, y_abs_comp, x_spot, y_spot, [x_spot[6], y_spot[6], z_spot[6]], x_offset, 1,
        theta_spot_updated, v_amp, step_phase, t1
    )
    x_framecenter_comp, y_framecenter_comp, z_framecenter_comp = calculate_frame_center(foot_center, compt, height)

    # Paso 9: Calcular posiciones de las patas
    pos = calculate_leg_positions(
        stance, t1, tstep, calculate_frame_corners(x_framecenter_comp, y_framecenter_comp, z_framecenter_comp, Ms_updated),
        x_spot, y_spot, z_spot, theta_spot_updated, Msi_updated, Ms_updated,
        xc, yc, radii, an, mangle, cw, v_amp, np.zeros(4), 1, x_offset, [x_spot[6], y_spot[6], z_spot[6]], np.zeros(4),
        x_spot, y_spot, Msi_comp, foot_center, x_framecenter_comp, y_framecenter_comp, z_framecenter_comp
    )

    return pos

def calculate_steering_center(steering_radius, steering_angle):
    """
    Calcula las coordenadas del centro de dirección.
    """
    xc = steering_radius * cos(steering_angle)
    yc = steering_radius * sin(steering_angle)
    return xc, yc

def transform_steering_center(Ms, xc, yc, x_spot, y_spot, z_spot):
    """
    Transforma las coordenadas del centro de dirección al marco del punto.
    """
    s = new_coordinates(Ms, xc, yc, 0, x_spot[0], y_spot[0], z_spot[0])
    return s[0], s[1]

def define_foot_positions(track):
    """
    Define las posiciones iniciales de las patas.
    """
    xn = [xlf, xrf, xrr, xlr]
    yn = [ylf + track, yrf - track, yrr - track, ylr + track]
    return xn, yn

def calculate_radii_and_angles(xc, yc, xn, yn):
    """
    Calcula los radios y ángulos de las patas respecto al centro de dirección.
    """
    radii = np.zeros(4)
    an = np.zeros(4)
    for i in range(4):
        radii[i] = sqrt((xc - xn[i]) ** 2 + (yc - yn[i]) ** 2)
        an[i] = atan2(yn[i] - yc, xn[i] - xc)
    return radii, an

def calculate_movement_angle(radii, h_amp):
    """
    Calcula el ángulo de movimiento basado en los radios y la amplitud horizontal.
    """
    maxr = max(radii)
    return h_amp / maxr

def calculate_rotation_angle(mangle, stepl, tstep, cw, step_phase, theta_spot):
    """
    Calcula el ángulo de rotación y traslación.
    """
    if step_phase in ('start', 'stop'):
        dtheta = mangle / (1 - stepl) * tstep / 2 * cw
    else:
        dtheta = mangle / (1 - stepl) * tstep * cw
    return dtheta

def update_rotation_matrices(theta_spot_updated, dtheta):
    """
    Actualiza las matrices de rotación para el marco del robot.
    """
    Ms_updated = xyz_rotation_matrix(theta_spot_updated[3], theta_spot_updated[4],
                                        theta_spot_updated[2] + theta_spot_updated[5], False)
    Msi_updated = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4],
                                        -(theta_spot_updated[2] + theta_spot_updated[5]), True)
    dMs = xyz_rotation_matrix(0, 0, dtheta, False)
    return Ms_updated, Msi_updated, dMs

def calculate_foot_center(dMs, xs, ys, x_spot, y_spot):
    """
    Calcula el centro de las patas después de aplicar la matriz de rotación.
    """
    return new_coordinates(dMs, x_spot[0] - xs, y_spot[0] - ys, 0, xs, ys, 0)

def initialize_stance_and_time(t, alphav, alpha, step_phase):
    """
    Inicializa las variables de apoyo para el ciclo de marcha.
    """
    stance = [True, True, True, True]
    t1 = t % 1
    for i in range(4):
        alphav[i] = 0
    return stance, t1

def calculate_support_area(x_spot, y_spot):
    """
    Calcula el área de soporte del robot.
    """
    x_abs_area = x_spot[2:6]
    y_abs_area = y_spot[2:6]
    return x_abs_area, y_abs_area

def calculate_transition_area(t1, stance_test, stance, x_abs_area, y_abs_area, stepl):
    """
    Calcula el área de transición durante el ciclo de marcha.
    """
    x_abs_comp = np.zeros(4)
    y_abs_comp = np.zeros(4)
    for i in range(4):
        if not stance_test[i]:
            x_abs_comp[i] = x_abs_area[i]
            y_abs_comp[i] = y_abs_area[i]
        elif t1 < (seq[i] + stepl):
            x_abs_comp[i] = x_abs_area[i] + (x_abs_area[(i + 1) % 4] - x_abs_area[i]) * (t1 - seq[i]) / stepl
            y_abs_comp[i] = y_abs_area[i] + (y_abs_area[(i + 1) % 4] - y_abs_area[i]) * (t1 - seq[i]) / stepl
    return x_abs_comp, y_abs_comp

def calculate_compensation(x_abs_comp, y_abs_comp, x_spot, y_spot, CG, x_offset, kcomp, theta_spot_updated,
                            v_amp, step_phase, t1):
    """
    Calcula la compensación para el movimiento del robot.
    """
    Msi_comp = xyz_rotation_matrix(0, 0, -theta_spot_updated[2], True)
    comp = new_coordinates(Msi_comp, x_abs_comp - x_spot[0], y_abs_comp - y_spot[0], 0, 0, 0, 0)

    v_amp_t = v_amp
    ts = 0.25
    if step_phase == 'start':
        if t1 < ts:
            kcomp = t1 / ts
            v_amp_t = 0
    elif step_phase == 'stop':
        if t1 > (1 - ts):
            kcomp = (1 - t1) / ts
            v_amp_t = 0

    Ms_comp = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)
    compt = new_coordinates(Ms_comp, (comp[0] - CG[0]) * kcomp + x_offset, (comp[1] - CG[1]) * kcomp, 0, 0, 0, 0)

    return comp, compt, v_amp_t, Msi_comp

def calculate_frame_center(foot_center, compt, height):
    """
    Calcula el centro del marco del robot.
    """
    x_framecenter_comp = foot_center[0] + compt[0]
    y_framecenter_comp = foot_center[1] + compt[1]
    z_framecenter_comp = height
    return x_framecenter_comp, y_framecenter_comp, z_framecenter_comp

def calculate_frame_corners(x_framecenter_comp, y_framecenter_comp, z_framecenter_comp, Ms_updated):
    """
    Calcula las esquinas del marco del robot.
    """
    x_frame = [xlf, xrf, xrr, xlr]
    y_frame = [ylf, yrf, yrr, ylr]
    z_frame = [0, 0, 0, 0]

    x_framecorner = np.zeros(4)
    y_framecorner = np.zeros(4)
    z_framecorner = np.zeros(4)

    for i in range(4):
        frame_corner = new_coordinates(Ms_updated, x_frame[i], y_frame[i], z_frame[i],
                                        x_framecenter_comp, y_framecenter_comp, z_framecenter_comp)
        x_framecorner[i] = frame_corner[0]
        y_framecorner[i] = frame_corner[1]
        z_framecorner[i] = frame_corner[2]

    return x_framecorner, y_framecorner, z_framecorner, x_frame, y_frame

def calculate_leg_positions(stance, t1, tstep, x_framecorner, y_framecorner, z_framecorner,
                            x_spot, y_spot, z_spot, theta_spot_updated, Msi_updated, Ms_updated,
                            xc, yc, radii, an, mangle, cw, v_amp, alphav, kcomp, x_offset, CG,
                            alpha, comp, x_frame, y_frame, Msi_comp, foot_center,
                            x_framecenter_comp, y_framecenter_comp, z_framecenter_comp):
    """
    Calcula las posiciones de las patas del robot.
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

            leg_current = new_coordinates(Msi_comp, x_spot[i + 2] - x_framecorner[i],
                                            y_spot[i + 2] - y_framecorner[i], -z_framecorner[i], 0, 0, 0)

            if (seq[i] + stepl - t1) > tstep:
                xabs[i] = leg_current[0] + (xleg_target - leg_current[0]) * tstep / (seq[i] + stepl - t1)
                yabs[i] = leg_current[1] + (yleg_target - leg_current[1]) * tstep / (seq[i] + stepl - t1)
            else:
                xabs[i] = xleg_target
                yabs[i] = yleg_target

            zabs[i] = leg_current[2] + v_amp * (1 + sin(alphav[i])) / 2

            Msi_body = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4],
                                            -theta_spot_updated[5], True)
            legs = new_coordinates(Msi_body, xabs[i], yabs[i], zabs[i], 0, 0, 0)
            xleg[i] = legs[0]
            yleg[i] = legs[1]
            zleg[i] = legs[2]

            foot_abs = new_coordinates(Ms_updated, xleg[i], yleg[i], zleg[i],
                                        x_framecorner[i], y_framecorner[i], z_framecorner[i])

            xabs[i] = foot_abs[0]
            yabs[i] = foot_abs[1]
            zabs[i] = foot_abs[2]
        else:
            xabs[i] = x_spot[i + 2]
            yabs[i] = y_spot[i + 2]
            zabs[i] = 0

            leg = new_coordinates(Msi_updated, xabs[i] - x_framecorner[i], yabs[i] - y_framecorner[i],
                                    zabs[i] - z_framecorner[i], 0, 0, 0)
            xleg[i] = leg[0]
            yleg[i] = leg[1]
            zleg[i] = leg[2]

    x_spot_updated = [foot_center[0], x_framecenter_comp, xabs[0], xabs[1], xabs[2], xabs[3], x_spot[6], x_spot[7],
                        x_spot[8]]
    y_spot_updated = [foot_center[1], y_framecenter_comp, yabs[0], yabs[1], yabs[2], yabs[3], y_spot[6], y_spot[7],
                        y_spot[8]]
    z_spot_updated = [foot_center[2], z_framecenter_comp, zabs[0], zabs[1], zabs[2], zabs[3], z_spot[6], z_spot[7],
                        z_spot[8]]

    pos = [xleg[0], yleg[0], zleg[0], xleg[1], yleg[1], zleg[1], xleg[2], yleg[2], zleg[2], xleg[3], yleg[3],
            zleg[3], theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated]

    return pos

