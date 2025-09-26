# -*- coding: utf-8 -*-  

"""
import numpy as np  # Librería para cálculos numéricos y matrices
import matplotlib.pyplot as plt  
import rospy  # Librería de ROS para manejar nodos y comunicación
from nav_msgs.msg import OccupancyGrid  # Importar mensaje de OccupancyGrid de ROS
from geometry_msgs.msg import PoseWithCovarianceStamped  # Importar mensaje de posición inicial

# Variables globales
grid_map = None  # Mapa de ocupación global, se actualizará con los datos recibidos del mapa
start_pose = None  # Posición inicial del robot, se actualizará con los datos recibidos del tópico

# Función para recibir y actualizar el OccupancyGrid en tiempo real
def occupancy_grid_callback(msg):
    global grid_map
    grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))  # Convertir OccupancyGrid en matriz bidimensional

# Función para recibir y actualizar la posición inicial del robot
def start_pose_callback(msg):
    global start_pose
    start_pose = (int(msg.pose.pose.position.x), int(msg.pose.pose.position.y))  # Extraer y almacenar la posición inicial del robot

# Función para dividir un área en una cuadrícula y verificar ocupación
def generate_grid_window(center, size_x, size_y, num_cells, occupancy_grid):
    cell_size_x = size_x / num_cells  # Calcular el tamaño de cada celda en la dirección x
    cell_size_y = size_y / num_cells  # Calcular el tamaño de cada celda en la dirección y
    free_cells = []  # Lista para almacenar las celdas libres
    
    for i in range(num_cells):  # Recorrer las filas de la cuadrícula
        for j in range(num_cells):  # Recorrer las columnas de la cuadrícula
            x = int(center[0] - size_x / 2 + i * cell_size_x)  # Calcular la coordenada x de la celda
            y = int(center[1] - size_y / 2 + j * cell_size_y)  # Calcular la coordenada y de la celda
            
            if 0 <= x < occupancy_grid.shape[0] and 0 <= y < occupancy_grid.shape[1]:  # Verificar si la celda está dentro del mapa
                if occupancy_grid[x, y] < 50:  # Celda libre si el valor es menor a 50 (basado en el umbral de ocupación)
                    free_cells.append((x, y))  # Agregar celda libre a la lista
    
    return free_cells  # Retornar la lista de celdas libres

# Función para generar una trayectoria basada en celdas libres
def generate_path(start, goal, free_cells):
    if not free_cells:  # Verificar si hay celdas libres disponibles
        print("No hay celdas libres disponibles para generar una trayectoria.")
        return [start]  # Retornar solo la posición inicial para evitar errores
    
    path = [start]  # Inicializar la trayectoria con la posición inicial
    current_pos = start  # Inicializar la posición actual
    
    while current_pos != goal and free_cells:  # Asegurar que haya celdas libres disponibles
        closest_cell = min(free_cells, key=lambda c: np.linalg.norm(np.array(c) - np.array(goal)))  # Seleccionar la celda más cercana al objetivo
        path.append(closest_cell)  # Agregar la celda a la trayectoria
        free_cells.remove(closest_cell)  # Remover la celda de la lista de celdas libres
        current_pos = closest_cell  # Actualizar la posición actual
        if np.linalg.norm(np.array(current_pos) - np.array(goal)) < 1:  # Verificar si se ha alcanzado el objetivo
            break  # Terminar el bucle si se alcanza el objetivo
    
    return path  # Retornar la trayectoria generada


# Función para visualizar la trayectoria y el mapa
def plot_path(grid_map, path):
    plt.figure(figsize=(8, 8))  # Crear una figura de tamaño 8x8
    plt.imshow(grid_map.T, origin='lower', cmap='gray', alpha=0.6)  # Mostrar el mapa de ocupación con transparencia
    
    path_x, path_y = zip(*path)  # Extraer coordenadas de la trayectoria
    plt.plot(path_x, path_y, "b-")  # Dibujar la trayectoria en color azul
    plt.plot(path[0][0], path[0][1], "go", markersize=10, label="Start")  # Marcar la posición inicial en verde
    plt.plot(path[-1][0], path[-1][1], "ro", markersize=10, label="Goal")  # Marcar la posición final en rojo
    
    plt.xlim(0, grid_map.shape[0])  # Establecer límites del eje x
    plt.ylim(0, grid_map.shape[1])  # Establecer límites del eje y
    plt.legend()  # Agregar leyenda al gráfico
    plt.show()  # Mostrar el gráfico

# Inicializar nodo de ROS y suscribirse a los tópicos de OccupancyGrid y posición inicial
rospy.init_node('path_planner')  # Inicializar el nodo de ROS
rospy.Subscriber('/grid_map', OccupancyGrid, occupancy_grid_callback)  # Suscribirse al mapa de ocupación
rospy.Subscriber('/vrep/pose_cov', PoseWithCovarianceStamped, start_pose_callback)  # Suscribirse a la posición inicial del robot

# Parámetros del área de búsqueda
size_x, size_y = 10, 10  # Definir el tamaño del recuadro alrededor del inicio
grid_resolution = 10  # Número de celdas en cada dirección para la cuadrícula de planificación

goal = (9, 9)  # Definir la posición objetivo

# Esperar actualización del mapa y posición inicial
rospy.sleep(2)  # Esperar a que ROS reciba los datos del mapa y la posición inicial
rospy.wait_for_message('/grid_map',OccupancyGrid)
print(grid_map)
if grid_map is not None and start_pose is not None:  # Verificar si se han recibido los datos
    free_cells = generate_grid_window(start_pose, size_x, size_y, grid_resolution, grid_map)  # Obtener celdas libres en la cuadrícula generada
    path = generate_path(start_pose, goal, free_cells)  # Generar la trayectoria basada en las celdas libres
    plot_path(grid_map, path)  # Graficar la trayectoria generada sobre el mapa de ocupación

"""

import numpy as np  # Librería para cálculos numéricos y matrices
import matplotlib.pyplot as plt 
import rospy  # Librería de ROS para manejar nodos y comunicación
from nav_msgs.msg import OccupancyGrid  # Importar mensaje de OccupancyGrid de ROS
from geometry_msgs.msg import PoseWithCovarianceStamped  # Importar mensaje de posición inicial

# Variables globales
grid_map = None  # Mapa de ocupación global, se actualizará con los datos recibidos del mapa
start_pose = None  # Posición inicial del robot, se actualizará con los datos recibidos del tópico
origin = None  # Origen del mapa en coordenadas globales
resolution = None  # Resolución del mapa

# Función para recibir y actualizar el OccupancyGrid en tiempo real
def occupancy_grid_callback(msg):
    global grid_map, origin, resolution
    grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))  # Convertir OccupancyGrid en matriz bidimensional
    resolution = msg.info.resolution  # Guardar la resolución del mapa
    origin = (msg.info.origin.position.x, msg.info.origin.position.y)  # Guardar el origen del mapa

# Función para recibir y actualizar la posición inicial del robot
def start_pose_callback(msg):
    global start_pose
    if origin is not None and resolution is not None:
        start_pose = (int((msg.pose.pose.position.x - origin[0]) / resolution),
                      int((msg.pose.pose.position.y - origin[1]) / resolution))  # Convertir coordenadas a celdas
    else:
        rospy.logwarn("Esperando a que se reciba el OccupancyGrid antes de procesar la posición inicial.")

# Función para dividir un área en una cuadrícula y verificar ocupación
def generate_grid_window(center, size_x, size_y, num_cells, occupancy_grid):
    cell_size_x = size_x / num_cells  # Calcular el tamaño de cada celda en la dirección x
    cell_size_y = size_y / num_cells  # Calcular el tamaño de cada celda en la dirección y
    considered_cells = []  # Lista para almacenar todas las celdas consideradas
    free_cells = []  # Lista para almacenar las celdas libres
    occupied_cells = []  # Lista para almacenar las celdas ocupadas
    for i in range(num_cells):  # Recorrer las filas de la cuadrícula
        for j in range(num_cells):  # Recorrer las columnas de la cuadrícula
            x = int((center[0] - size_x / 2) / resolution + i + origin[0] / resolution)
            y = int((center[1] - size_y / 2) / resolution + j + origin[1] / resolution)
            considered_cells.append((x, y))  # Agregar celda a la lista de consideradas
            if 0 <= x < occupancy_grid.shape[0] and 0 <= y < occupancy_grid.shape[1]:  # Verificar si la celda está dentro del mapa
                if occupancy_grid[x, y] < 50:  # Celda libre si el valor es menor a 50 (basado en el umbral de ocupación)
                    free_cells.append((x, y))  # Agregar celda libre a la lista
                else:
                    occupied_cells.append((x, y))  # Agregar celda ocupada a la lista
    
    return considered_cells, free_cells, occupied_cells  # Retornar la lista de celdas consideradas, libres y ocupadas

# Función para visualizar las celdas libres y ocupadas
def plot_cells(grid_map, considered_cells, free_cells, occupied_cells):
    plt.figure(figsize=(8, 8))  # Crear una figura de tamaño 8x8
    plt.imshow(grid_map.T, origin='lower', cmap='gray', alpha=0.6)  # Mostrar el mapa de ocupación con transparencia
    
    if considered_cells:
        cons_x, cons_y = zip(*considered_cells)  # Extraer coordenadas de celdas consideradas
        plt.scatter(cons_x, cons_y, color='blue', label="Considered Cells", marker='s', alpha=0.3)  # Marcar celdas consideradas en azul claro
    
    if free_cells:
        free_x, free_y = zip(*free_cells)  # Extraer coordenadas de celdas libres
        plt.scatter(free_x, free_y, color='green', label="Free Cells", marker='s')  # Marcar celdas libres en verde
    
    if occupied_cells:
        occ_x, occ_y = zip(*occupied_cells)  # Extraer coordenadas de celdas ocupadas
        plt.scatter(occ_x, occ_y, color='red', label="Occupied Cells", marker='s')  # Marcar celdas ocupadas en rojo
    
    plt.plot(start_pose[0], start_pose[1], "bo", markersize=10, label="Start")  # Marcar la posición inicial en azul
    
    plt.xlim(0, grid_map.shape[0])  # Establecer límites del eje x
    plt.ylim(0, grid_map.shape[1])  # Establecer límites del eje y
    plt.legend()  # Agregar leyenda al gráfico
    plt.show()  # Mostrar el gráfico

# Inicializar nodo de ROS y suscribirse a los tópicos de OccupancyGrid y posición inicial
rospy.init_node('cell_visualizer')  # Inicializar el nodo de ROS
rospy.Subscriber('/grid_map', OccupancyGrid, occupancy_grid_callback)  # Suscribirse al mapa de ocupación
rospy.wait_for_message('/grid_map',OccupancyGrid)

rospy.Subscriber('/vrep/pose_cov', PoseWithCovarianceStamped, start_pose_callback)  # Suscribirse a la posición inicial del robot

# Parámetros del área de búsqueda
size_x, size_y = 10, 10  # Definir el tamaño del recuadro alrededor del inicio
grid_resolution = 10  # Número de celdas en cada dirección para la cuadrícula de análisis

# Esperar actualización del mapa y posición inicial
rospy.sleep(2)  # Esperar a que ROS reciba los datos del mapa y la posición inicial

if grid_map is not None and start_pose is not None:  # Verificar si se han recibido los datos
    considered_cells, free_cells, occupied_cells = generate_grid_window(start_pose, size_x, size_y, grid_resolution, grid_map)  # Obtener celdas consideradas, libres y ocupadas
    plot_cells(grid_map, considered_cells, free_cells, occupied_cells)  # Graficar las celdas sobre el mapa de ocupación
