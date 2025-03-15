import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Definir la función de transformación
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

# Matriz de rotación (rotación de 90 grados alrededor del eje Z)
theta = np.pi / 2
rotation_matrix = np.array([
    [np.cos(theta), -np.sin(theta), 0],
    [np.sin(theta), np.cos(theta), 0],
    [0, 0, 1]
])

# Coordenadas iniciales de los puntos
x_array = np.array([1, 0, 0])
y_array = np.array([0, 1, 0])
z_array = np.array([0, 0, 1])

# Vector de traslación
dx, dy, dz = 1, 2, 3

# Aplicar la transformación
new_x, new_y, new_z = new_coordinates_vec(rotation_matrix, x_array, y_array, z_array, dx, dy, dz)

# Crear una figura 3D para graficar los puntos
fig = plt.figure(figsize=(12, 6))

# Subplot 1: Puntos originales
ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(x_array, y_array, z_array, c='blue', s=100, label='Puntos originales')
# Conectar los puntos originales como un triángulo
ax1.plot(np.append(x_array, x_array[0]), 
         np.append(y_array, y_array[0]), 
         np.append(z_array, z_array[0]), 
         c='blue', linestyle='--', label='Triángulo original')

# Enumerar los puntos originales
for i, (x, y, z) in enumerate(zip(x_array, y_array, z_array)):
    ax1.text(x, y, z, f' {i+1}', color='black', fontsize=12)

# Dibujar ejes de coordenadas en el origen
ax1.quiver(0, 0, 0, 1.5, 0, 0, color='red', label='Eje X')  # Eje X
ax1.quiver(0, 0, 0, 0, 1.5, 0, color='green', label='Eje Y')  # Eje Y
ax1.quiver(0, 0, 0, 0, 0, 1.5, color='blue', label='Eje Z')  # Eje Z

ax1.set_title("Puntos Originales")
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
ax1.legend()

# Subplot 2: Puntos transformados
ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(new_x, new_y, new_z, c='red', s=100, label='Puntos transformados')
# Conectar los puntos transformados como un triángulo
ax2.plot(np.append(new_x, new_x[0]), 
         np.append(new_y, new_y[0]), 
         np.append(new_z, new_z[0]), 
         c='red', linestyle='--', label='Triángulo transformado')

# Enumerar los puntos transformados
for i, (x, y, z) in enumerate(zip(new_x, new_y, new_z)):
    ax2.text(x, y, z, f' {i+1}', color='black', fontsize=12)

# Dibujar ejes de coordenadas en el origen
ax2.quiver(0, 0, 0, 1.5, 0, 0, color='red', label='Eje X')  # Eje X
ax2.quiver(0, 0, 0, 0, 1.5, 0, color='green', label='Eje Y')  # Eje Y
ax2.quiver(0, 0, 0, 0, 0, 1.5, color='blue', label='Eje Z')  # Eje Z

ax2.set_title("Puntos Transformados")
ax2.set_xlabel("X'")
ax2.set_ylabel("Y'")
ax2.set_zlabel("Z'")
ax2.legend()

# Mostrar la gráfica
plt.tight_layout()
plt.show()