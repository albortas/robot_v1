import numpy as np
from src.utils.operador import xyz_rotation_matrix, new_coordinates
from src.config.puntos_torso import *

""""
Moving Function from known start and end positions (used for sitting, lying, etc...)
"""

def moving(t, start_frame_pos, end_frame_pos, pos):
    theta_spot_updated = pos[12]
    x_spot_updated = pos[13]
    y_spot_updated = pos[14]
    z_spot_updated = pos[15]

    frame_pos = interpolate_frame_position(start_frame_pos, end_frame_pos, t)

    theta_spot_updated[3] = frame_pos[0]
    theta_spot_updated[4] = frame_pos[1]
    theta_spot_updated[5] = frame_pos[2]

    Mf = xyz_rotation_matrix(frame_pos[0], frame_pos[1], frame_pos[2], False)
    Ms = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)

    x_frame = [xlf, xrf, xrr, xlr]
    y_frame = [ylf, yrf, yrr, ylr]
    z_frame = [0, 0, 0, 0]

    frame_center_abs = new_coordinates(Ms, frame_pos[3], frame_pos[4], frame_pos[5],
                                        x_spot_updated[0], y_spot_updated[0], z_spot_updated[0])

    x_frame_corner_abs, y_frame_corner_abs, z_frame_corner_abs = calculate_absolute_frame_corners(
        Mf, Ms, x_frame, y_frame, z_frame, frame_center_abs)

    xleg, yleg, zleg = calculate_relative_leg_positions(theta_spot_updated, x_frame_corner_abs,
                                                                y_frame_corner_abs, z_frame_corner_abs, x_spot_updated,
                                                                y_spot_updated, z_spot_updated)

    x_spot_updated[1] = frame_center_abs[0]
    y_spot_updated[1] = frame_center_abs[1]
    z_spot_updated[1] = frame_center_abs[2]

    pos = [xleg[0], yleg[0], zleg[0], xleg[1], yleg[1], zleg[1], xleg[2], yleg[2], zleg[2], xleg[3], yleg[3],
            zleg[3], theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated]

    return pos

def interpolate_frame_position(start_frame_pos, end_frame_pos, t):
    frame_pos = np.zeros(6)
    for i in range(0, 6):
        frame_pos[i] = start_frame_pos[i] + (end_frame_pos[i] - start_frame_pos[i]) * t
    return frame_pos

def calculate_absolute_frame_corners(Mf, Ms, x_frame, y_frame, z_frame, frame_center_abs):
    x_frame_corner_abs = np.zeros(4)
    y_frame_corner_abs = np.zeros(4)
    z_frame_corner_abs = np.zeros(4)

    for i in range(0, 4):
        frame_corner = new_coordinates(Mf, x_frame[i], y_frame[i], z_frame[i], 0, 0, 0)
        frame_corner_abs = new_coordinates(Ms, frame_corner[0], frame_corner[1], frame_corner[2],
                                            frame_center_abs[0], frame_center_abs[1], frame_center_abs[2])
        x_frame_corner_abs[i] = frame_corner_abs[0]
        y_frame_corner_abs[i] = frame_corner_abs[1]
        z_frame_corner_abs[i] = frame_corner_abs[2]

    return x_frame_corner_abs, y_frame_corner_abs, z_frame_corner_abs

def calculate_relative_leg_positions(theta_spot_updated, x_frame_corner_abs, y_frame_corner_abs,
                                        z_frame_corner_abs, x_spot_updated, y_spot_updated, z_spot_updated):
    Mi = xyz_rotation_matrix(-theta_spot_updated[3], -theta_spot_updated[4],
                                -(theta_spot_updated[2] + theta_spot_updated[5]), True)

    xleg = np.zeros(4)
    yleg = np.zeros(4)
    zleg = np.zeros(4)

    for i in range(0, 4):
        leg = new_coordinates(Mi, x_spot_updated[i + 2] - x_frame_corner_abs[i],
                                y_spot_updated[i + 2] - y_frame_corner_abs[i],
                                z_spot_updated[i + 2] - z_frame_corner_abs[i], 0, 0, 0)
        xleg[i] = leg[0]
        yleg[i] = leg[1]
        zleg[i] = leg[2]

    return xleg, yleg, zleg