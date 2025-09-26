#!/usr/bin/env python

#import roslib;roslib.load_manifest("RosExamples")
import rospy
import sys
import tf
from geometry_msgs .msg import Twist
from sensor_msgs.msg import LaserScan
from math import tanh
from std_msgs.msg import Float32


class Stopper():
    
    def __init__(self):
        
        rospy.init_node("stopper", anonymous=False)
        self.leftmotor = rospy.Publisher("/leftMotorSpeed",Float32,queue_size=10)
        self.rightmotor = rospy.Publisher("/rightMotorSpeed",Float32,queue_size=10)
        self.leftmotor.publish(0)
        self.rightmotor.publish(0)
        self.keyboard()
        
            
    def keyboard(self):
        left=0
        right=0
        while not rospy.is_shutdown():
            self.leftmotor.publish(left)
            self.rightmotor.publish(right)
            #nd = raw_input()
            nd=raw_input()
            if nd[0]!='w' and nd[0]!='s' and nd[0]!='a' and nd[0]!='d' and nd[0]!='.':
                rospy.loginfo("Comando incorrecto %s"%nd[0])
                continue 
            if nd[0]=='w':
                left=5
                right=5
            if nd[0]=='s':
                left=-5
                right=-5
            if nd[0]=='a':
                right=5
                left=1
            if nd[0]=='d':
                left=5
                right=1
            if nd[0]=='.':
                left=0
                right=0
        
def main(args):
    try:
        Stopper()
        rospy.spin()
    except KeyboardInterrupt:
        print "Finalizando Stopper."


if __name__ == '__main__':
    main(sys.argv)
