import numpy as np

def xyz_rotation_matrix(thetax, thetay, thetaz, inverse = False):
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
    rot4x4[:3,:3] = xyz_rotation_matrix(alpha,beta,gamma,inversa)
    return homog_transxyz(dx, dy, dz) @ rot4x4



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

def new_coordinates(M, x, y, z, dx = 0, dy = 0, dz = 0):
    """
    Transforma punto usando una matriz de rotación y una traslación.
    """
    point = np.array([x, y, z])
    offset = np.array([dx, dy, dz])
    transformed_point = M @ point.T + offset.T
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
    translated = rotated + np.vstack([dx, dy, dz])
    
    # Devolver como tres arrays separados
    return translated[0, :], translated[1, :], translated[2, :]

def new_coord_full(x, y, z, alpha, beta, gamma,inversa = False, dx = 0, dy = 0, dz = 0):
    """
    Transforma punto usando una angulos y una traslación.
    """
    matriz = xyz_rotation_matrix(alpha, beta, gamma, inversa)
    punto = np.array([x, y, z])
    p_org = np.array([dx, dy, dz])
    M = matriz @ punto + p_org
    return M


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
    import time
    
    alpha = pi/2
    beta = pi/2
    gamma = pi/2
    #dx, dy, dz = 1, 1, 1
    dx = dy = dz = np.array([1,1,1,1])
    x=y=z = np.array([1,1, 1,5])
    
    ini = time.time()
    mate = xyz_rotation_matrix(alpha, beta, gamma)
    M = new_coordinates_vec(mate, x,y,z, dx,dy,dz)
    fin = time.time()
    
    print(M)
    print(dx-x)
    #print(fin-ini)

    