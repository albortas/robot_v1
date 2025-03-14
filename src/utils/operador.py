import numpy as np
from math import sin, cos

def xyz_rotation_matrix(thetax, thetay, thetaz, inverse):
    cx, sx = np.cos(thetax), np.sin(thetax)
    cy, sy = np.cos(thetay), np.sin(thetay)
    cz, sz = np.cos(thetaz), np.sin(thetaz)

    if inverse:
        # Rzyx = Rx * Ry * Rz
        M = np.array([
            [cy * cz, -sz * cy, sy],
            [sx * sy * cz + sz * cx, -sx * sy * sz + cx * cz, -sx * cy],
            [sx * sz - sy * cx * cz, sx * cz + sy * sz * cx, cx * cy]
        ])
    else:
        # Rxyz = Rz * Ry * Rx
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

def new_coordinates_vec(rotation_matrix, x_array, y_array, z_array, dx=0, dy=0, dz=0):
    """
    Transforma múltiples puntos usando una matriz de rotación y una traslación.
    """
    # Apilar coordenadas en una matriz (3, n)
    points = np.stack([x_array, y_array, z_array])
    
    # Aplicar rotación: (3x3) @ (3xn) -> (3xn)
    rotated = np.dot(rotation_matrix, points)
    
    # Aplicar traslación (broadcasting automático)
    translated = rotated + np.array([[dx], [dy], [dz]])
    
    # Devolver como tres arrays separados
    return translated[0, :], translated[1, :], translated[2, :]

def rotx(alpha):
    """
    Create a 3x3 rotation matrix about the x axis
    """
    rx = np.array([[1, 0,          0,         ],
                   [0, cos(alpha), -sin(alpha)],
                   [0, sin(alpha), cos(alpha) ]])

    return rx


def roty(beta):
    """
    Create a 3x3 rotation matrix about the y axis
    """
    ry = np.array([[cos(beta),   0, sin(beta)],
                   [0,           1, 0        ],
                   [-sin(beta),  0, cos(beta)]])

    return ry


def rotz(gamma):
    """
    Create a 3x3 rotation matrix about the z axis
    """
    rz = np.array([[cos(gamma), -sin(gamma), 0],
                   [sin(gamma), cos(gamma),  0],
                   [0,          0,           1]])

    return rz


def rotxyz(alpha, beta, gamma, inversa):
    """
    Create a 3x3 rotation matrix about the x,y,z axes
    """
    #Rzyx = Rx * Ry * Rz
    if inversa:
        return rotx(alpha).dot(roty(beta)).dot(rotz(gamma))
    #Rxyz = Rz * Ry * Rx
    else:
        return rotz(gamma) @ roty(beta) @ rotx(alpha)


def homog_transxyz(dx, dy, dz):
    """
    Create a 4x4 homogeneous translation matrix (translation along the x,y,z axes)
    """
    trans = np.array([[1, 0, 0, dx],
                      [0, 1, 0, dy],
                      [0, 0, 1, dz],
                      [0, 0, 0, 1 ]])
    return trans


def homog_transform(alpha,beta,gamma,inversa,dx,dy,dz):
    """
    Crea una transformada homogenea 4x4 transformation matrix
                 ------------------------- 
                 |           |           |  
           T  =  |     R     |    Porg   | 
                 |___________|___________| 
                 | 0   0   0 |     1     | 
                 -------------------------
    """    
      
    rot4x4 = np.eye(4)
    rot4x4[:3,:3] = rotxyz(alpha,beta,gamma,inversa)
    return homog_transxyz(dx, dy, dz) @ rot4x4

def nuev_coord_trans(alpha, beta, gamma, inversa, x, y, z, dx, dy, dz):
    coord = homog_transform(alpha, beta, gamma, inversa, dx, dy, dz) @ np.array([x,y,z,0])
    return coord[0:3]


def homog_transform_inverse(matrix):
    """
    Return the inverse of a homogeneous transformation matrix.

                 ------------------------- 
                 |           |           |  
    inverse   =  |    R^T    |  -R^T * d | 
                 |___________|___________| 
                 | 0   0   0 |     1     | 
                 -------------------------  

    """
    inverse = matrix
    inverse[:3,:3] = inverse[:3,:3].T # R^T
    inverse[:3,3] = -np.dot(inverse[:3,:3],inverse[:3,3]) # -R^T * d
    return inverse

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

if __name__ == "__main__":
    from math import pi
    alpha = pi/2
    beta = pi/2
    gamma = pi/2
    dx, dy, dz = 1, 2, 3
    x,y,z = 7, 8, 9
    
    M = nuev_coord_trans(alpha,beta,gamma, False, x,y,z, dx,dy,dz)
    print(M)

    