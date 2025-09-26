#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid

class SlopeOccupancyPublisher:
    def __init__(self):
        # Parámetros configurables
        self.min_slope = rospy.get_param("~min_slope", 0.0)
        self.max_slope = rospy.get_param("~max_slope", 30.0)

        # Subscripción al GridMap y publicación del OccupancyGrid
        self.sub = rospy.Subscriber('/elevation_mapping/elevation_map_raw', GridMap, self.callback)
        self.pub = rospy.Publisher("slope_occupancy", OccupancyGrid, queue_size=1)

    def callback(self, msg):
        # Verificar que la capa "slope" exista en el mensaje
        if "slope" not in msg.layers:
            rospy.logwarn("La capa 'slope' no se encuentra en el grid_map recibido")
            return

        # Se asume que msg.info contiene: resolution, width, height y pose (origin)
        width = msg.info.length_x
        height = msg.info.length_y
        resolution = msg.info.resolution
        origin = msg.info.pose

        num_cells = width * height
        num_layers = len(msg.layers)
        print(len(msg.data))
        # Verificar que el tamaño de 'data' sea consistente
        if len(msg.data) != num_layers * num_cells:
            rospy.logwarn("El tamaño de 'data' no es consistente con las dimensiones del grid_map")
            return

        try:
            layer_index = msg.layers.index("slope")
        except ValueError:
            rospy.logwarn("La capa 'slope' no se encontró en layers")
            return

        # Extraer los datos correspondientes a la capa "slope"
        start_idx = layer_index * num_cells
        end_idx = start_idx + num_cells
        slope_data = msg.data[start_idx:end_idx]

        # Convertir los valores de slope a valores de ocupación:
        # - NaN se marca como desconocido (-1)
        # - Valores <= min_slope se asignan 0 (libre)
        # - Valores >= max_slope se asignan 100 (ocupado)
        # - Valores intermedios se interpolan linealmente.
        occupancy_data = []
        for value in slope_data:
            if math.isnan(value):
                occ = -1
            else:
                if value <= self.min_slope:
                    occ = 0
                elif value >= self.max_slope:
                    occ = 100
                else:
                    occ = int(round((value - self.min_slope) / (self.max_slope - self.min_slope) * 100))
            occupancy_data.append(occ)

        # Construir el mensaje OccupancyGrid
        occ_grid = OccupancyGrid()
        occ_grid.header = msg.header
        occ_grid.info.resolution = resolution
        occ_grid.info.width = width
        occ_grid.info.height = height
        occ_grid.info.origin = origin
        occ_grid.data = occupancy_data

        self.pub.publish(occ_grid)

if __name__ == '__main__':
    rospy.init_node("slope_occupancy_publisher")
    node = SlopeOccupancyPublisher()
    rospy.spin()
