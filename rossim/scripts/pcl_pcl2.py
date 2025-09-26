#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2,PointCloud
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

rospy.init_node('pcl2_pub')

class Variables_(object):
    
    cloud_points=[]
    header=''

variables=Variables_()

class Publishers_(object):
    
    pcl_pub = rospy.Publisher("velodyne/points2", PointCloud2,queue_size=1)
    
pubs=Publishers_()

class Funciones_(object):
    
    def pcl_callback(self,msg):
                
        variables.header=msg.header.frame_id
        
        variables.cloud_points=[]
        
        for point in msg.points:
            
            variables.cloud_points.append([point.x,point.y,point.z])
            
        self.pub_pcl2()
        
    def pub_pcl2(self):
        
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = variables.header
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, variables.cloud_points)
        pubs.pcl_pub.publish(scaled_polygon_pcl)
        
funciones=Funciones_()

class Subscriptores(object):
    
    rospy.Subscriber('velodyne/scan',PointCloud,funciones.pcl_callback)
    
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
