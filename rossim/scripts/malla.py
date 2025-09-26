#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, math, tf
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import *
from geometry_msgs.msg import *
from dynamic_reconfigure.server import Server
from rossim.cfg import _rossim_Config

rospy.init_node('grid_path_planner', anonymous=False)

class _Variables_(object):
    grid = None
    pose_cov = None
    trayectoria = []
    candidatos_libres = []
    candidatos_ocupados = []
    init_x = 0.0
    init_y = 0.0
    resolution = None
    origin_x = None
    origin_y = None
    spacing_cm = 100
    search_radius_cells = 20
    turn_weight = 0.0
    path_activo = []  # Nueva trayectoria activa

variables = _Variables_()

class _Funciones_(object):

    def callback(self, config, level):
        rospy.loginfo("Updated config: %s", config)
        variables.spacing_cm = config.spacing_cm
        variables.search_radius_cells = config.search_radius_cells
        variables.turn_weight = config.turn_weight
        return config

    # Algoritmo de Bresenham para interpolar una línea entre dos celdas
    
    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def is_free(self, cell, grid):
        x, y = cell
        if x < 0 or x >= grid.info.width or y < 0 or y >= grid.info.height:
            return False
        index = y * grid.info.width + x
        value = grid.data[index]
        return value == 0

    def normalizar_angulo(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_yaw_from_quaternion(self, orientation):
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def grid_callback(self, msg):
        variables.grid = msg

    def pose_callback(self, msg):
        variables.pose_cov = msg

funciones = _Funciones_()

class Subscribers(object):
    rospy.Subscriber('grid_map', OccupancyGrid, funciones.grid_callback)
    rospy.Subscriber('vrep/pose_cov', PoseWithCovarianceStamped, funciones.pose_callback)

class Publishers(object):
    path_pub = rospy.Publisher('trayectoria', Path, queue_size=1)

pubs = Publishers()

class GridPathPlanner(object):

    def __init__(self):
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.timer = rospy.Timer(rospy.Duration(10.0), self.timer_callback)

    def timer_callback(self, event):
        if variables.grid is None or variables.pose_cov is None:
            rospy.loginfo("Esperando mapa y pose inicial...")
            return

        variables.resolution = variables.grid.info.resolution
        variables.origin_x = variables.grid.info.origin.position.x
        variables.origin_y = variables.grid.info.origin.position.y

        init_pose = variables.pose_cov.pose.pose
        variables.init_x = init_pose.position.x
        variables.init_y = init_pose.position.y
        current_yaw = funciones.get_yaw_from_quaternion(init_pose.orientation)

        init_cell_x = int((variables.init_x - variables.origin_x) / variables.resolution)
        init_cell_y = int((variables.init_y - variables.origin_y) / variables.resolution)

        spacing = variables.spacing_cm / 100.0
        spacing_cells = int(round(spacing / variables.resolution))

        # Verificar si la trayectoria actual ha sido alcanzada o bloqueada
        traj_finished = not variables.path_activo
        for cell in variables.path_activo:
            if not funciones.is_free(cell, variables.grid):
                traj_finished = True
                break

        if not traj_finished:
            rospy.loginfo("Trayectoria actual sigue siendo válida. No se recalcula.")
            return

        best_score = -1e9
        best_path = None
        best_distance = 0.0
        candidatos_libres = []
        candidatos_ocupados = []

        # Recorrer celdas en una ventana cuadrada alrededor de la celda inicial
        
        for dx in range(-int(variables.search_radius_cells), int(variables.search_radius_cells) + 1):
            for dy in range(-int(variables.search_radius_cells), int(variables.search_radius_cells) + 1):
                candidato_x = init_cell_x + dx * spacing_cells
                candidato_y = init_cell_y + dy * spacing_cells
                candidato = (candidato_x, candidato_y)

                # Convertir la celda candidata a coordenadas del mundo (centro de la celda)
                
                candidato_mundo_x = candidato_x * variables.resolution + variables.origin_x + variables.resolution / 2.0
                candidato_mundo_y = candidato_y * variables.resolution + variables.origin_y + variables.resolution / 2.0

                # Registrar el punto según su estado en el OccupancyGrid
                
                if funciones.is_free(candidato, variables.grid):
                    candidatos_libres.append((candidato_mundo_x, candidato_mundo_y))
                else:
                    candidatos_ocupados.append((candidato_mundo_x, candidato_mundo_y))

                # Para planificación solo se consideran celdas libres
                
                if not funciones.is_free(candidato, variables.grid):
                    continue

                # Verificar que la línea recta entre la celda inicial y la candidata esté libre
                linea = funciones.bresenham(init_cell_x, init_cell_y, candidato_x, candidato_y)
                num_ocupados = sum(1 for cell in linea if not funciones.is_free(cell, variables.grid))
                if num_ocupados > 5:  # puedes ajustar este umbral
                    continue

                # Calcular la distancia entre la pose inicial y el punto candidato (en metros)
                distance = math.sqrt((candidato_mundo_x - variables.init_x)**2 + (candidato_mundo_y - variables.init_y)**2)
                desired_yaw = math.atan2(candidato_mundo_y - variables.init_y, candidato_mundo_x - variables.init_x)
                turning_diff = abs(funciones.normalizar_angulo(desired_yaw - current_yaw))
                candidate_score = distance - variables.turn_weight * turning_diff

                if candidate_score > best_score:
                    best_score = candidate_score
                    best_path = linea
                    best_distance = distance

        if best_path is None:
            rospy.loginfo("No se encontró una trayectoria válida.")
            trayectoria = []
        else:
            # Conversión de la trayectoria (si se encontró) a coordenadas del mundo
            rospy.loginfo("Trayectoria encontrada. Puntuacion: {:.2f}, Distancia: {:.2f} m.".format(best_score, best_distance))
            trayectoria = []
            for cell in best_path:
                x = cell[0] * variables.resolution + variables.origin_x + variables.resolution / 2.0
                y = cell[1] * variables.resolution + variables.origin_y + variables.resolution / 2.0
                trayectoria.append((x, y))
                rospy.loginfo("Punto: x = {:.2f}, y = {:.2f}".format(x, y))

        # Generación de la trayectoria final
        
        variables.trayectoria = trayectoria
        variables.path_activo = best_path if best_path else []
        variables.candidatos_libres = candidatos_libres
        variables.candidatos_ocupados = candidatos_ocupados

        # Publicación del mensaje de trayectoria en formato Path
        
        if trayectoria:
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = variables.grid.header.frame_id if variables.grid.header.frame_id else "map"
            for point in trayectoria:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = point[0]
                pose_stamped.pose.position.y = point[1]
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)
            pubs.path_pub.publish(path_msg)

    def update_plot(self):
        if variables.grid is None or variables.resolution is None:
            return

        self.ax.clear()
        grid_data = np.array(variables.grid.data).reshape((variables.grid.info.height, variables.grid.info.width))
        extent = [variables.origin_x, variables.origin_x + variables.grid.info.width * variables.resolution,
                  variables.origin_y, variables.origin_y + variables.grid.info.height * variables.resolution]
        self.ax.imshow(grid_data, cmap='gray', origin='lower', extent=extent)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Mapa, Candidatos y Trayectoria')

        # Dibujar celdas ocupadas en trayectoria activa (si las hay)
        for cell in variables.path_activo:
            if not funciones.is_free(cell, variables.grid):
                x = cell[0] * variables.resolution + variables.origin_x + variables.resolution / 2.0
                y = cell[1] * variables.resolution + variables.origin_y + variables.resolution / 2.0
                self.ax.plot(x, y, 'ms', markersize=8)
                self.ax.text(x, y, "Obstaculo", color='magenta', fontsize=8, ha='center', va='bottom')
                break

        if variables.candidatos_libres:
            free_np = np.array(variables.candidatos_libres)
            self.ax.scatter(free_np[:, 0], free_np[:, 1], c='g', marker='o', label='Candidatos libres')

        if variables.candidatos_ocupados:
            occ_np = np.array(variables.candidatos_ocupados)
            self.ax.scatter(occ_np[:, 0], occ_np[:, 1], c='r', marker='x', label='Candidatos ocupados')

        self.ax.plot(variables.init_x, variables.init_y, 'bo', markersize=10, label='Pose inicial')
        self.ax.plot(variables.init_x, variables.init_y, 'ro', markersize=4, label='Progreso')
        
        if variables.trayectoria:
            # Trazo completo de la trayectoria en azul claro si ya fue recorrida
            traj_np = np.array(variables.trayectoria)
            self.ax.plot(traj_np[:, 0], traj_np[:, 1], color='lightblue', linewidth=2, label='Trayectoria recorrida')

            # Marcar el punto final
            last_point = variables.trayectoria[-1]
            dist_to_goal = math.hypot(last_point[0] - variables.init_x, last_point[1] - variables.init_y)
            color_final = 'green' if dist_to_goal < 0.2 else 'cyan'
            self.ax.plot(last_point[0], last_point[1], 'o', color=color_final, markersize=8,
                            label='Punto final alcanzado' if color_final == 'green' else 'Punto final')
            self.ax.text(last_point[0], last_point[1], "Final", color=color_final, fontsize=8, ha='center', va='top')

            # Mostrar la parte futura pendiente de recorrer
            reached_index = 0
            for i in range(len(traj_np)):
                dist = math.hypot(traj_np[i, 0] - variables.init_x, traj_np[i, 1] - variables.init_y)
                if dist < 0.2:
                    reached_index = i
                    break
            if reached_index + 1 < len(traj_np):
                self.ax.plot(traj_np[reached_index:, 0], traj_np[reached_index:, 1], '--', color='orange', linewidth=1.5, label='Trayectoria pendiente')

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

if __name__ == '__main__':
    try:
        planner = GridPathPlanner()
        srv = Server(_rossim_Config, funciones.callback)
        while not rospy.is_shutdown():
            variables.spacing_cm = rospy.get_param('/gridmap/spacing_cm')
            variables.search_radius_cells = rospy.get_param('/gridmap/search_radius_cells')
            variables.turn_weight = rospy.get_param('/gridmap/turn_weight')
            planner.update_plot()
            plt.pause(0.01)
    except rospy.ROSInterruptException as e:
        print(e)
        pass