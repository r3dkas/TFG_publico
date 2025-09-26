#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <grid_map_msgs/GridMap.h>
#include "cv_bridge/cv_bridge.h"
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_publisher.h>

// Gridmap
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

#include <dynamic_reconfigure/server.h>
#include <rossim/_rossim_Config.h>

ros::Publisher map_pub;
double_t limite_minimo;
double_t limite_maximo;

void callback(rossim::_rossim_Config &config, uint32_t level) {
    limite_minimo=config.minimo;
    limite_maximo=config.maximo;
}

void elevation_callback(const grid_map_msgs::GridMap& map)
{
    ROS_INFO("Reconfigure Request: %f %f", limite_minimo,limite_maximo);
    ROS_INFO("Received scanned information");
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(map,gridMap);
    nav_msgs::OccupancyGrid occupancy_map;
    grid_map::GridMapRosConverter::toOccupancyGrid(gridMap, "traversability", limite_minimo, limite_maximo, occupancy_map);
    map_pub.publish(occupancy_map);
}

int main (int argc, char** argv) {

    // Initialize ROS
    ROS_INFO("Node started");

    ros::init(argc, argv, "gridmap");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<rossim::_rossim_Config> server;
    dynamic_reconfigure::Server<rossim::_rossim_Config>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Subscriber sub = nh.subscribe("/elevation_mapping/elevation_map_raw", 1, elevation_callback);

    map_pub = nh.advertise<nav_msgs::OccupancyGrid> ("/grid_map", 1);

    ros::spin();
    return 0;
}