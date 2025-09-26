#!/usr/bin/env python
import rospy,tf
from sensor_msgs.msg import PointCloud2,PointCloud
import std_msgs.msg
from nav_msgs.msg import *
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import *
from math import *

rospy.init_node('pose_cov_nodo')

class Variables_(object):
    
    pose_cov=PoseWithCovarianceStamped()
    
variables=Variables_()

class Publishers_(object):
    
    pose_cov_pub = rospy.Publisher("vrep/pose_cov", PoseWithCovarianceStamped,queue_size=1)
    odom=rospy.Publisher("/odom",Odometry,queue_size=1)
    
pubs=Publishers_()

class Funciones_(object):
    
    def pose_callback(self,msg):
                         
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        
        variables.pose_cov.header=header
        
        variables.pose_cov.pose.pose.position=msg.position
        
        variables.pose_cov.pose.pose.position.z=0
        
        _,_,yaw = tf.transformations.euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        
        quaternion=tf.transformations.quaternion_from_euler(0,0,yaw+pi/2)
        
        variables.pose_cov.pose.pose.orientation.x=quaternion[0]
        variables.pose_cov.pose.pose.orientation.y=quaternion[1]
        variables.pose_cov.pose.pose.orientation.z=quaternion[2]
        variables.pose_cov.pose.pose.orientation.w=quaternion[3]
        odom=Odometry()
        
        odom.header=header
        odom.pose=variables.pose_cov.pose

        pubs.pose_cov_pub.publish(variables.pose_cov)
        pubs.odom.publish(odom)
        
funciones=Funciones_()

class Subscriptores(object):
    
    rospy.Subscriber('vrep/robot_pose',Pose,funciones.pose_callback)
    
class Nodo(object):
    
    def __init__(self):
        pass
    
    def run(self):
        
        while not rospy.is_shutdown():
            
            rospy.spin()

if __name__ == '__main__':

    rospy.sleep(1.)
    nodo=Nodo()
    nodo.run()

    