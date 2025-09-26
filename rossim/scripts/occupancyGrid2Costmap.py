#!/usr/bin/env python

import rospy
from nav_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np

class OccupancyGridToCostmap:
    def __init__(self):
        rospy.init_node('occupancy_grid_to_costmap')
        self.map_sub = rospy.Subscriber('/grid_map', OccupancyGrid, self.map_callback)
        self.costmap_pub = rospy.Publisher('/costmap', OccupancyGrid, queue_size=1)
        self.map_metadata = None

    def map_callback(self, msg):
        if self.map_metadata is None:
            self.map_metadata = msg.info
        self.transform_to_costmap(msg)

    def transform_to_costmap(self, occupancy_grid):
        costmap = OccupancyGrid()
        costmap.header.frame_id = "map"
        #costmap.info.resolution = 0.05  # Resolucin del mapa (metros por celda)
        #costmap.info.width = 100  # Ancho del mapa en celdas
        #costmap.info.height = 100  # Alto del mapa en celdas
        #costmap.info.origin.position.x = 0.0  # Posicin del origen del mapa
        #costmap.info.origin.position.y = 0.0
        #costmap.info.origin.position.z = 0.0
        #costmap.info.origin.orientation.w = 1.0

        #costmap.header = occupancy_grid.header
        costmap.info = occupancy_grid.info
        costmap.header.stamp = rospy.Time.now()
        costmap.data = self.generate_costmap(occupancy_grid)
        
        rospy.loginfo(costmap.info.origin)
        self.costmap_pub.publish(costmap)

    def generate_costmap(self, occupancy_grid):
        resolution = self.map_metadata.resolution
        width = self.map_metadata.width
        height = self.map_metadata.height

        costmap_data = np.zeros(width * height, dtype=np.int8)

        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if occupancy_grid.data[idx] == 100:
                    costmap_data[idx] = 100
                else:
                    costmap_data[idx] = 0

        return costmap_data

if __name__ == '__main__':
    try:
        occupancy_grid_to_costmap = OccupancyGridToCostmap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
