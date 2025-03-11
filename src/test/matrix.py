import src.utils.operador as op
import numpy as np

thetax = np.radians(30)  # Rotación alrededor de X
thetay = np.radians(50)  # Rotación alrededor de Y
thetaz = np.radians(80)  # Rotación alrededor de Z

# Punto inicial
x, y, z = 1, 1, 1

# Desplazamiento
x0, y0, z0 = 2, 2, 2

# Matriz de rotación
M = op.xyz_rotation_matrix(thetax, thetay, thetaz, inverse=False)
print(M)

# Nuevas coordenadas
new_coords = op.new_coordinates(M, x, y, z, x0, y0, z0)
print("Nuevas coordenadas:", new_coords)

# Coordenadas del pie
foot_coords = op.foot_coordinate(x, y, z, thetax, thetay)
print("Coordenadas del pie:", foot_coords)