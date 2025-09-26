#!/usr/bin/env python

import rospy,ros,tf2_ros,tf
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from grid_map_msgs.msg import *
from tf2_msgs.msg import *
from tf.broadcaster import TransformBroadcaster


class Elevacion(object):
    
    def __init__(self):
        
        self.elevacion=0
        self.rate = rospy.Rate(10.0)
        self.inicio=False
        
        rospy.Subscriber('/vrep/robot_pose', Pose , self.callback)

    def callback(self,msg):
        
        self.elevacion=msg.position.z
        self.inicio=True

    def main(self):
        
        while not rospy.is_shutdown():
            
            if self.inicio:
                
                br = tf2_ros.TransformBroadcaster()
                transform = geometry_msgs.msg.TransformStamped()

                transform.header.frame_id = "mapa_terreno"
                transform.child_frame_id = "map"
                transform.transform.translation.x = 0.0
                transform.transform.translation.y = 0.0
                transform.transform.translation.z = -self.elevacion  # dynamically changing height
                transform.transform.rotation.x = 0.0
                transform.transform.rotation.y = 0.0
                transform.transform.rotation.z = 0.0
                transform.transform.rotation.w = 1.0
                
                transform.header.stamp = rospy.Time.now()
                br.sendTransform(transform)
                self.rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node('elevacion_map')
    
    elevacion=Elevacion()
    elevacion.main()