from src.utils.operador import xyz_rotation_matrix, new_coordinates, foot_coordinate
import numpy as np

thetax = np.radians(30)  # Rotación alrededor de X
thetay = np.radians(45)  # Rotación alrededor de Y
thetaz = np.radians(60)  # Rotación alrededor de Z

# Punto inicial
x, y, z = 1, 0, 0

# Desplazamiento
x0, y0, z0 = 0, 0, 0

# Matriz de rotación
M = xyz_rotation_matrix(thetax, thetay, thetaz, inverse=False)
print(M)

# Nuevas coordenadas
new_coords = new_coordinates(M, x, y, z, x0, y0, z0)
print("Nuevas coordenadas:", new_coords)

# Coordenadas del pie
foot_coords = foot_coordinate(x, y, z, thetax, thetay)
print("Coordenadas del pie:", foot_coords)