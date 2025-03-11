import numpy as np

def xyz_rotation_matrix(thetax, thetay, thetaz, inverse):
    cx, sx = np.cos(thetax), np.sin(thetax)
    cy, sy = np.cos(thetay), np.sin(thetay)
    cz, sz = np.cos(thetaz), np.sin(thetaz)

    if inverse:
        # Rx * Ry * Rz
        M = np.array([
            [cy * cz, -sz * cy, sy],
            [sx * sy * cz + sz * cx, -sx * sy * sz + cx * cz, -sx * cy],
            [sx * sz - sy * cx * cz, sx * cz + sy * sz * cx, cx * cy]
        ])
    else:
        # Rz * Ry * Rx
        M = np.array([
            [cy * cz, sx * sy * cz - sz * cx, sx * sz + sy * cx * cz],
            [sz * cy, sx * sy * sz + cx * cz, -sx * cz + sy * sz * cx],
            [-sy, sx * cy, cx * cy]
        ])
    return M

def new_coordinates(M, x, y, z, x0, y0, z0):
    point = np.array([x, y, z])
    offset = np.array([x0, y0, z0])
    transformed_point = M @ point  + offset
    return transformed_point.tolist()

def foot_coordinate(x, y, z, thetax, thetay):
    cx, sx = np.cos(thetax), np.sin(thetax)
    cy, sy = np.cos(thetay), np.sin(thetay)

    M = np.array([
        [cy, 0, sy],
        [sx * sy, cx, -sx * cy],
        [-cx * sy, sx, cx * cy]
    ])

    point = np.array([x, y, z])
    return (M @ point).tolist()

