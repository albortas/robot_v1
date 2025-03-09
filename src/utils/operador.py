from math import sin,cos

def xyz_rotation_matrix(thetax, thetay, thetaz, inverse):
    if inverse:
        # Rx*Ry*Rz
        t2 = cos(thetay)
        t3 = sin(thetaz)
        t4 = cos(thetaz)
        t5 = sin(thetay)
        t6 = cos(thetax)
        t7 = sin(thetax)
        M = [t2 * t4, t3 * t6 + t4 * t5 * t7, t3 * t7 - t4 * t5 * t6, -t2 * t3, t4 * t6 - t3 * t5 * t7,
             t4 * t7 + t3 * t5 * t6, t5, -t2 * t7, t2 * t6]
    else:
        # Rz*Ry*Rx
        t2 = cos(thetaz)
        t3 = sin(thetax)
        t4 = sin(thetaz)
        t5 = cos(thetax)
        t6 = sin(thetay)
        t7 = cos(thetay)
        M = [t2 * t7, t4 * t7, -t6, -t4 * t5 + t2 * t3 * t6, t2 * t5 + t3 * t4 * t6, t3 * t7, t3 * t4 + t2 * t5 * t6,
             -t2 * t3 + t4 * t5 * t6, t5 * t7]
    return M


def new_coordinates(M, x, y, z, x0, y0, z0):
    xout = x0 + M[0] * x + M[3] * y + M[6] * z
    yout = y0 + M[1] * x + M[4] * y + M[7] * z
    zout = z0 + M[2] * x + M[5] * y + M[8] * z
    return [xout, yout, zout]


def foot_coordinate(x, y, z, thetax, thetay):
    cx = cos(thetax)
    sx = sin(thetax)
    cy = cos(thetay)
    sy = sin(thetay)
    xf = x * cy + z * sy
    yf = y * cx - z * cy * sx + x * sx * sy
    zf = y * sx + z * cx * cy - x * cx * sy
    return [xf, yf, zf]

