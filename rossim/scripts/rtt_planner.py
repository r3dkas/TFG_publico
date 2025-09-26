# -*- coding: utf-8 -*-


import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import rospy
from nav_msgs.msg import OccupancyGrid

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def distance(p1, p2):
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def generate_random_point(x_max, y_max):
    return np.random.uniform(0, x_max), np.random.uniform(0, y_max)

def nearest_node(tree, point):
    distances, index = tree.query(point)
    return index

def steer(from_node, to_point, step_size):
    theta = np.arctan2(to_point[1] - from_node.y, to_point[0] - from_node.x)
    new_x = from_node.x + step_size * np.cos(theta)
    new_y = from_node.y + step_size * np.sin(theta)
    return Node(new_x, new_y)

def check_collision_grid(new_node, occupancy_grid):
    x, y = int(new_node.x), int(new_node.y)
    if 0 <= x < occupancy_grid.shape[0] and 0 <= y < occupancy_grid.shape[1]:
        return occupancy_grid[x, y] > 50  # Considerar ocupacion si el valor es mayor a 50
    return True  # Considerar fuera del mapa como obstaculo

def occupancy_grid_callback(msg):
    global grid_map
    grid_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))

def rrt(start, goal, x_max, y_max, step_size, max_iter, occupancy_grid):
    nodes = [Node(start[0], start[1])]
    for _ in range(max_iter):
        rand_point = generate_random_point(x_max, y_max)
        tree = KDTree([(node.x, node.y) for node in nodes])
        nearest_idx = nearest_node(tree, rand_point)
        new_node = steer(nodes[nearest_idx], rand_point, step_size)
        
        if not check_collision_grid(new_node, occupancy_grid):
            new_node.parent = nodes[nearest_idx]
            nodes.append(new_node)
            
            if distance((new_node.x, new_node.y), goal) < step_size:
                print("Goal reached!")
                return nodes
    return nodes

def plot_rrt(nodes, start, goal, occupancy_grid):
    plt.figure(figsize=(8, 8))
    plt.imshow(occupancy_grid.T, origin='lower', cmap='gray', alpha=0.6)
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "b-")
    
    plt.plot(start[0], start[1], "go", markersize=10, label="Start")
    plt.plot(goal[0], goal[1], "ro", markersize=10, label="Goal")
    
    plt.xlim(0, occupancy_grid.shape[0])
    plt.ylim(0, occupancy_grid.shape[1])
    plt.legend()
    plt.show()

# Inicializar ROS y suscribirse al mapa de ocupación
rospy.init_node('rrt_planner')
grid_map = np.zeros((500, 500))

rospy.Subscriber('/grid_map', OccupancyGrid, occupancy_grid_callback)

# Parámetros
start = (1, 1)
goal = (450, 450)
x_max, y_max = 500, 500
step_size = 0.5
max_iter = 500

# Esperar el mapa antes de ejecutar RRT
rospy.sleep(2)

# Ejecutar RRT
nodes = rrt(start, goal, x_max, y_max, step_size, max_iter, grid_map)

# Graficar resultado
plot_rrt(nodes, start, goal, grid_map)
