import numpy as np
from src.utils.operador import xyz_rotation_matrix, new_coordinates, new_coord_full
from src.config.puntos_torso import *

""""
Moving Function from known start and end positions (used for sitting, lying, etc...)
"""

def moving(t, start_frame_pos, end_frame_pos, pos):
    theta_spot_updated = pos[12]
    x_spot_updated = pos[13]
    y_spot_updated = pos[14]
    z_spot_updated = pos[15]

    frame_pos = np.array(start_frame_pos[:6]) + (np.array(end_frame_pos[:6]) - (start_frame_pos[:6])) * t
    

    theta_spot_updated[3] = frame_pos[0]
    theta_spot_updated[4] = frame_pos[1]
    theta_spot_updated[5] = frame_pos[2]

    x_frame = [xlf, xrf, xrr, xlr]
    y_frame = [ylf, yrf, yrr, ylr]
    z_frame = [0, 0, 0, 0]

    Ms = xyz_rotation_matrix(0, 0, theta_spot_updated[2], False)
    frame_center_abs = new_coordinates(Ms, frame_pos[3], frame_pos[4], frame_pos[5],
                                        x_spot_updated[0], y_spot_updated[0], z_spot_updated[0])
    
    """ frame_center_abs = new_coord_full(frame_pos[3], frame_pos[4], frame_pos[5],
                                      0, 0, theta_spot_updated[2], False,
                                      x_spot_updated[0], y_spot_updated[0], z_spot_updated[0]) """
    
    x_frame_corner, y_frame_corner, z_frame_corner = new_coord_full(x_frame, y_frame, z_frame,frame_pos[0], frame_pos[1], frame_pos[2])
    x_frame_corner_abs, y_frame_corner_abs, z_frame_corner_abs = new_coord_full(x_frame_corner, y_frame_corner, z_frame_corner,
                                                                                0, 0, theta_spot_updated[2], False, 
                                                                                frame_center_abs[0], frame_center_abs[1], frame_center_abs[2])

    xleg, yleg, zleg = new_coord_full(x_spot_updated[2:6] - x_frame_corner_abs, y_spot_updated[2:6] - y_frame_corner_abs,
                                      z_spot_updated[2:6] - z_frame_corner_abs, -theta_spot_updated[3], -theta_spot_updated[4],
                                      -(theta_spot_updated[2] + theta_spot_updated[5]), True)

    x_spot_updated[1] = frame_center_abs[0]
    y_spot_updated[1] = frame_center_abs[1]
    z_spot_updated[1] = frame_center_abs[2]

    pos = [xleg[0], yleg[0], zleg[0], xleg[1], yleg[1], zleg[1], xleg[2], yleg[2], zleg[2], xleg[3], yleg[3],
            zleg[3], theta_spot_updated, x_spot_updated, y_spot_updated, z_spot_updated]

    return pos





