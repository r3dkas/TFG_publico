#!/usr/bin/env python
import rospy
import numpy as np
import math
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion

# Parameters for slope-to-cost conversion:
# Cells with slope <= SLOPE_LOW will be free (cost 0)
# Cells with slope >= SLOPE_HIGH will be obstacles (cost 100)
SLOPE_LOW = 0.0   # radians
SLOPE_HIGH = 0.5  # radians (adjust as needed)

def map_slope_to_cost(slope_value):
    if np.isnan(slope_value):
        return -1  # Unknown cell
    if slope_value <= SLOPE_LOW:
        return 0
    if slope_value >= SLOPE_HIGH:
        return 100
    cost = 100.0 * (slope_value - SLOPE_LOW) / (SLOPE_HIGH - SLOPE_LOW)
    return int(round(cost))

def reorder_circular_buffer(data_2d, outer_start, inner_start):
    """
    Reorders a 2D array stored in a circular buffer.
    'data_2d' is of shape [outer_size, inner_size], where typically
      dim[0] (columns) = outer_size and dim[1] (rows) = inner_size.
    The 'outer_start' and 'inner_start' offsets are applied to "unwrap" the buffer.
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
    # Attempt to read circular buffer offsets; default to 0 if not present.
    outer_start = getattr(layout_info, 'outer_start_index', 90)
    inner_start = getattr(layout_info, 'inner_start_index', 155)

    if len(layout_info.dim) >= 2:
        # In many grid_map messages, dim[0] (label "column_index") holds number of columns,
        # and dim[1] (label "row_index") holds number of rows.
        outer_size = layout_info.dim[0].size  # columns
        inner_size = layout_info.dim[1].size  # rows
    else:
        resolution = msg.info.resolution
        outer_size = int(round(msg.info.length_x / resolution))
        inner_size = int(round(msg.info.length_y / resolution))

    try:
        data_2d = data_array.reshape((outer_size, inner_size))
    except Exception as e:
        rospy.logerr("Error reshaping data: %s", e)
        return

    # Reorder the data to account for the circular buffer
    corrected_2d = reorder_circular_buffer(data_2d, outer_start, inner_start)
    # Rotate clockwise 90 degrees to correct the orientation.
    rotated_2d = np.rot90(corrected_2d, k=2)

    # After rotation, the shape is:
    new_rows, new_cols = rotated_2d.shape

    # Generate costmap data by mapping each slope value to a cost.
    cost_data = []
    for i in range(new_rows):
        for j in range(new_cols):
            slope_val = rotated_2d[i, j]
            cost = map_slope_to_cost(slope_val)
            cost_data.append(cost)

    # Create an OccupancyGrid message.
    occ_grid = OccupancyGrid()
    occ_grid.header.stamp = rospy.Time.now()
    occ_grid.header.frame_id = msg.info.header.frame_id  # Should match your map frame

    occ_grid.info.resolution = msg.info.resolution
    occ_grid.info.width = new_cols
    occ_grid.info.height = new_rows

    # Compute new map dimensions based on the rotated grid.
    new_length_x = new_cols * occ_grid.info.resolution
    new_length_y = new_rows * occ_grid.info.resolution

    # The grid map message's pose.position is the center of the original map.
    center_x = msg.info.pose.position.x
    center_y = msg.info.pose.position.y

    # Set the origin of the costmap to be the lower-left corner.
    origin_x = center_x - (new_length_x / 2.0)
    origin_y = center_y - (new_length_y / 2.0)
    occ_grid.info.origin = Pose()
    occ_grid.info.origin.position = Point(origin_x, origin_y, 0.0)
    occ_grid.info.origin.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    occ_grid.data = cost_data

    costmap_pub.publish(occ_grid)
    rospy.loginfo("Published slope costmap with dimensions %d x %d", new_cols, new_rows)

def main():
    global costmap_pub
    rospy.init_node("slope_costmap_generator")
    costmap_pub = rospy.Publisher("slope_costmap", OccupancyGrid, queue_size=1)
    rospy.Subscriber("elevation_mapping/elevation_map_raw", GridMap, grid_map_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
