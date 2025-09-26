 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "laser_geometry.h"





void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
ros::Publisher pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "las2pcl");
  ros::NodeHandle n;
  std::string topic = n.resolveName("point_cloud");
    
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  Messages are passed to a callback function, here
   * called cmd_dirCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
  **/
  ros::Subscriber sub = n.subscribe("/base_scan3d", 1000, scanCallback);

  //pub = n.advertise<sensor_msgs/PointCloud2>("/pointcloud", PointCloud2);
  pub = n.advertise<sensor_msgs::PointCloud>("/pointcloud", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  //ROS_INFO("Received scanned information);
  //geometry_msgs::Twist sent_message;
  //double linear=0, angular=0;
      laser_geometry::LaserProjection projector_;
        sensor_msgs::PointCloud cloud;
        
      projector_.projectLaser(*scan_in, cloud);

  //ROS_INFO_STREAM("Value " << sent_message.linear.x << " " << sent_message.angular.z);
  pub.publish(cloud);

}
