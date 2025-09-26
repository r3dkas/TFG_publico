#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from grid_map_msgs.msg import GridMap

# Variable global para almacenar el objeto de la imagen
img_handle = None
plt.ion()  # Modo interactivo

def grid_map_callback(msg):
    # Imprime las capas disponibles
    rospy.loginfo("Available layers: %s", msg.layers)
    
    # Define la capa que se desea extraer
    target_layer = "slope"
    if target_layer not in msg.layers:
        rospy.logwarn("Layer %s not found in grid map.", target_layer)
        return

    # Obtiene el indice de la capa y extrae la lista de datos plana
    idx = msg.layers.index(target_layer)
    flat_data = msg.data[idx].data

    # Obtiene la geometria del grid map del mensaje
    resolution = msg.info.resolution
    length_x = msg.info.length_x
    length_y = msg.info.length_y

    # Calcula el numero de celdas en x e y
    num_cells_x = int(round(length_x / resolution))
    num_cells_y = int(round(length_y / resolution))
    rospy.loginfo("Resolution: %.3f m, Size: %.2f x %.2f m, Cells: %d x %d",
                  resolution, length_x, length_y, num_cells_x, num_cells_y)

    # Convierte la lista plana a un arreglo numpy y lo reorganiza en una matriz 2D
    try:
        data_array = np.array(flat_data, dtype=np.float32)
        grid = data_array.reshape((num_cells_y, num_cells_x))
    except Exception as e:
        rospy.logerr("Error reshaping data: %s", e)
        return

    rospy.loginfo("Layer %s extracted with shape %s", target_layer, grid.shape)
    rospy.loginfo("Values: min=%.3f, max=%.3f", np.nanmin(grid), np.nanmax(grid))
    
    # Plotea la capa en tiempo real
    global img_handle
    if img_handle is None:
        plt.figure("Grid Map Layer")
        img_handle = plt.imshow(grid, cmap="viridis", origin="lower", interpolation="none")
        plt.colorbar()
        plt.title("Layer: " + target_layer)
    else:
        img_handle.set_data(grid)
        img_handle.set_clim(np.nanmin(grid), np.nanmax(grid))
    plt.draw()
    plt.pause(0.001)

def main():
    rospy.init_node("grid_map_extractor")
    rospy.Subscriber("/elevation_mapping/elevation_map_raw", GridMap, grid_map_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
