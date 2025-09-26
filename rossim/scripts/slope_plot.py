#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from grid_map_msgs.msg import GridMap

img_handle = None

def reorder_circular_buffer(data_2d, outer_start, inner_start):
    """
    Reorders a 2D array 'data_2d' (shape [outer_size, inner_size])
    according to the circular buffer offsets 'outer_start' and 'inner_start'.
    Returns a new 2D array of the same shape, but 'unwrapped'.
    """
    outer_size, inner_size = data_2d.shape
    corrected = np.empty_like(data_2d)
    for i in range(outer_size):
        for j in range(inner_size):
            row = (outer_start + i) % outer_size
            col = (inner_start + j) % inner_size
            corrected[i, j] = data_2d[row, col]
    return corrected

def grid_map_callback(msg):
    target_layer = "slope"
    if target_layer not in msg.layers:
        rospy.logwarn("Layer '%s' not found.", target_layer)
        return

    layer_idx = msg.layers.index(target_layer)
    try:
        flat_data = msg.data[layer_idx].data
        data_array = np.array(flat_data, dtype=np.float32)
    except Exception as e:
        rospy.logerr("Error extracting slope data: %s", e)
        return

    layout_info = msg.data[layer_idx].layout
    # Some versions of grid_map_msgs do not have these offsets
    outer_start = getattr(layout_info, 'outer_start_index', 90)
    inner_start = getattr(layout_info, 'inner_start_index', 155)

    # Determine the dimensions (columns vs rows).
    if len(layout_info.dim) >= 2:
        outer_size = layout_info.dim[0].size
        inner_size = layout_info.dim[1].size
    else:
        # fallback if layout not set
        resolution = msg.info.resolution
        outer_size = int(round(msg.info.length_x / resolution))
        inner_size = int(round(msg.info.length_y / resolution))

    try:
        # Reshape into [outer_size, inner_size]
        data_2d = data_array.reshape((outer_size, inner_size))
    except Exception as e:
        rospy.logerr("Error reshaping data: %s", e)
        return

    # If your map uses a circular buffer, reorder the data to remove "split lines".
    corrected_2d = reorder_circular_buffer(data_2d, outer_start, inner_start)

    # Now rotate the result CCW by 90 degrees:
    rotated_2d = np.rot90(corrected_2d, k=3)  # k=1 => 90 deg counter-clockwise

    global img_handle
    if img_handle is None:
        plt.figure("Slope Layer (CCW 90 deg)")
        img_handle = plt.imshow(rotated_2d, cmap="jet", origin="lower", interpolation="none")
        plt.colorbar()
        plt.title("Slope Layer (Circular Buffer Corrected & Rot90 CCW)")
    else:
        img_handle.set_data(rotated_2d)
        img_handle.set_clim(np.nanmin(rotated_2d), np.nanmax(rotated_2d))
    plt.draw()
    plt.pause(0.001)

def main():
    rospy.init_node("slope_layer_unwrap_plotter")
    rospy.Subscriber("elevation_mapping/elevation_map_raw", GridMap, grid_map_callback)
    plt.ion()
    plt.show()
    rospy.spin()

if __name__ == '__main__':
    main()
