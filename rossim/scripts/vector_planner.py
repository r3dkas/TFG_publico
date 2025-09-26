# -*- coding: utf-8 -*-  


import numpy as np  # Librería para cálculos numéricos
import matplotlib.pyplot as plt

#from __future__ import unicode_literals
from scipy.spatial import KDTree  # Para buscar vecinos más cercanos eficientemente
import rospy  # Librería de ROS para comunicación con otros nodos
from nav_msgs.msg import OccupancyGrid  # Tipo de mensaje para mapas de ocupación en ROS


# Clase Node para representar un nodo en el árbol RRT
class Node:
    def __init__(self, x, y):
        self.x = x  # Coordenada X del nodo
        self.y = y  # Coordenada Y del nodo
        self.parent = None  # Nodo padre para reconstruir la ruta

# Función para calcular la distancia euclidiana entre dos puntos
def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Función para generar un punto aleatorio dentro de los límites del mapa
def generate_random_point(x_max, y_max):
    return np.random.uniform(0, x_max), np.random.uniform(0, y_max)  # Punto aleatorio en [0, x_max] y [0, y_max]

# Función para encontrar el nodo más cercano en el árbol RRT usando KDTree
def nearest_node(tree, point):
    distances, index = tree.query(point)  # Encuentra el nodo más cercano al punto aleatorio
    return index  # Devuelve el índice del nodo más cercano

# Función para crear un nuevo nodo en la dirección de un punto objetivo (`steer`)
def steer(from_node, to_point, step_size):
    theta = np.arctan2(to_point[1] - from_node.y, to_point[0] - from_node.x)  # Ángulo hacia el punto objetivo
    new_x = from_node.x + step_size * np.cos(theta)  # Nueva coordenada X en la dirección del ángulo
    new_y = from_node.y + step_size * np.sin(theta)  # Nueva coordenada Y en la dirección del ángulo
    return Node(new_x, new_y)  # Devuelve el nuevo nodo generado

# Función para verificar si un nodo está en una celda ocupada del mapa
def check_collision_grid(new_node, occupancy_grid):
    x, y = int(new_node.x), int(new_node.y)  # Convierte coordenadas flotantes a enteras (índices de la matriz)
    if 0 <= x < occupancy_grid.shape[0] and 0 <= y < occupancy_grid.shape[1]:  # Verifica si está dentro de los límites
        return occupancy_grid[x, y] > 50  # Considera ocupado si el valor en la celda es mayor a 50
    return True  # Si está fuera del mapa, se considera como obstáculo

# Función de callback para actualizar el mapa de ocupación cuando llega un mensaje de ROS
def occupancy_grid_callback(msg):
    global grid_map  # Variable global que almacena el mapa
    grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))  # Convierte los datos del mensaje en una matriz

# Función principal del algoritmo RRT
def rrt(start, goal, x_max, y_max, step_size, max_iter, occupancy_grid):
    nodes = [Node(start[0], start[1])]  # Lista de nodos, inicializada con el nodo de inicio
    for _ in range(max_iter):  # Itera hasta el número máximo de iteraciones
        rand_point = generate_random_point(x_max, y_max)  # Genera un punto aleatorio en el espacio
        tree = KDTree([(node.x, node.y) for node in nodes])  # Construye un KDTree para búsqueda rápida
        nearest_idx = nearest_node(tree, rand_point)  # Encuentra el nodo más cercano al punto aleatorio
        new_node = steer(nodes[nearest_idx], rand_point, step_size)  # Genera un nuevo nodo en esa dirección
        
        if not check_collision_grid(new_node, occupancy_grid):  # Si no hay colisión con obstáculos
            new_node.parent = nodes[nearest_idx]  # Asigna el nodo padre al nuevo nodo
            nodes.append(new_node)  # Agrega el nuevo nodo a la lista
            
            if distance((new_node.x, new_node.y), goal) < step_size:  # Si el nodo está cerca de la meta
                print("Goal reached!")  # Imprime que se alcanzó la meta
                return nodes  # Devuelve la lista de nodos como resultado
    return nodes  # Devuelve la lista de nodos si no encuentra la meta

# Función para graficar el árbol RRT y el mapa de ocupación
def plot_rrt(nodes, start, goal, occupancy_grid):
    plt.figure(figsize=(8, 8))  # Define el tamaño de la figura
    plt.imshow(occupancy_grid.T, origin='lower', cmap='gray', alpha=0.6)  # Muestra el mapa de ocupación en escala de grises
    
    for node in nodes:  # Recorre todos los nodos generados
        if node.parent:  # Si el nodo tiene un padre (no es el nodo inicial)
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "b-")  # Dibuja una línea entre el nodo y su padre
    
    plt.plot(start[0], start[1], "go", markersize=10, label="Start")  # Dibuja el punto de inicio en verde
    plt.plot(goal[0], goal[1], "ro", markersize=10, label="Goal")  # Dibuja el punto meta en rojo
    
    plt.xlim(0, occupancy_grid.shape[0])  # Ajusta los límites del eje X
    plt.ylim(0, occupancy_grid.shape[1])  # Ajusta los límites del eje Y
    plt.legend()  # Muestra la leyenda
    plt.show()  # Muestra la gráfica

# Inicialización de ROS y suscripción al mapa de ocupación
rospy.init_node('rrt_planner')  # Inicializa el nodo de ROS llamado 'rrt_planner'
grid_map = np.zeros((500, 500))  # Inicializa un mapa de ocupación vacío (10x10)
rospy.Subscriber('/costmap', OccupancyGrid, occupancy_grid_callback)  # Se suscribe al tópico '/map'

# Definición de parámetros de la planificación RRT
start = (1, 1)  # Coordenadas del punto de inicio
goal = (450, 450)  # Coordenadas del punto objetivo
x_max, y_max = 500, 500  # Dimensiones del espacio de búsqueda
step_size = 0.5  # Tamaño del paso entre nodos
max_iter = 500  # Número máximo de iteraciones

# Esperar la llegada del mapa antes de ejecutar el algoritmo
rospy.sleep(2)  # Espera 2 segundos para asegurar que el mapa de ocupación ha sido recibido

rospy.wait_for_message('/costmap',OccupancyGrid)

# Ejecutar RRT y graficar el resultado
nodes = rrt(start, goal, x_max, y_max, step_size, max_iter, grid_map)  # Ejecuta el algoritmo RRT
plot_rrt(nodes, start, goal, grid_map)  # Grafica el árbol RRT y el mapa de ocupación
