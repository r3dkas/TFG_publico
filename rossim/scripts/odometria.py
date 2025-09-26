#! /usr/bin/python

import rospy,tf,tf2_ros
import sys,os,subprocess,math,traceback,csv
from std_msgs.msg import *
import numpy as np
from math import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from tf.broadcaster import TransformBroadcaster

class odometria(object):


    def __init__(self):

        ########____PARAMETROS____########

        self.encoder_min = 0
        self.encoder_max = 4294967295
        ##self.pulsos_vuelta=7916.81
        ##self.diametro_rueda=0.280
        self.dist_entre_ruedas= 0.522639 
        self.pulsos_por_metro= 10182.6 
        
        self.dist_entre_ruedas_ale=0.522639 
        self.pulsos_por_metro_ale=10182.6 

        self.encoder_low_wrap = ((self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min ) ## limites para comprobar si se ha dado la vuelta 
        self.encoder_high_wrap = ((self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min ) ## idem
        
        self.frecuencia_tf=30
        self.t_delta = rospy.Duration(1.0/self.frecuencia_tf)
        self.t_next = rospy.Time.now() + self.t_delta
        
        ##################################
    
        # internal data
        
        self.pulsos_antIZQ= 0 
        self.pulsos_antDER= 0 

        self.pulsos_antIZQ_ale=int
        self.pulsos_antDER_ale=int
        self.pulsos_IZQ_ale=int
        self.pulsos_DER_ale=int

        self.pulsos_IZQ=0               
        self.pulsos_DER=0 
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                 
        self.y = 0
        self.th = 0
        self.dx = 0                 
        self.dr = 0
        self.then = rospy.Time.now() 
        self.posth=0
        self.posy=0
        self.posx=0
        
        self.x_ale=0
        self.y_ale=0
        self.th_ale=0
        
        self.antes=rospy.Time.now()
        
        self.enc_der=0
        self.enc_izq=0

        self.enc_der_ale=0
        self.enc_izq_ale=0
        
        self.start=[False]*5
        self.i=0
          
        #################################################

        # subscriptions
        self.tl = tf.TransformListener()

        self.base_frame_id='base_link'
        self.odom_frame_id = 'odom'

        rospy.Subscriber("/lwheel", UInt32, self.lwheelCallback)
        rospy.Subscriber("/rwheel", UInt32, self.rwheelCallback)
        
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=1)
        self.odomStamped = rospy.Publisher("odom_PoseStamped",Pose2D,queue_size=1)
        self.pub_time=rospy.Publisher('tiempoOdom',Float32,queue_size=1)
        self.pub_pulsos_izq=rospy.Publisher('ODOMETRIA/pulsos_izq',Float32,queue_size=1)
        self.pub_pulsos_der=rospy.Publisher('ODOMETRIA/pulsos_der',Float32,queue_size=1)
        self.pub_pulsos_izq_ale=rospy.Publisher('ODOMETRIA/pulsos_izq_ale',Float32,queue_size=1)
        self.pub_pulsos_der_ale=rospy.Publisher('ODOMETRIA/pulsos_der_ale',Float32,queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()
    
    ### funciones para comprobar si se ha reiniciado la cuenta##
    
    def lwheelCallback(self, msg):
        
        enc = msg.data
            
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.pulsos_IZQ_ale=(enc +self.encoder_max -self.prev_lencoder)
            self.lmult = self.lmult + 1
            
        elif (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.pulsos_IZQ_ale=(enc - self.encoder_max - self.prev_lencoder)
            self.lmult = self.lmult - 1
            
        else:
            self.pulsos_IZQ_ale=enc-self.prev_lencoder 
            
        self.pulsos_IZQ = enc + self.lmult * (self.encoder_max - self.encoder_min)
        self.prev_lencoder = enc
        
        self.i+=1
        
    def rwheelCallback(self, msg):
        
        enc = msg.data
                
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.pulsos_DER_ale=(enc +self.encoder_max -self.prev_rencoder)
            self.rmult = self.rmult + 1
        
        elif(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.pulsos_DER_ale=(enc -self.encoder_max -self.prev_rencoder)
            self.rmult = self.rmult - 1
        
        else:
            self.pulsos_DER_ale=enc-self.prev_rencoder
            
        self.pulsos_DER = enc + self.rmult * (self.encoder_max - self.encoder_min)
        self.prev_rencoder = enc
        
        self.i+=1
    
    def update(self):
        
        now = rospy.Time.now()

        if now > self.t_next:                                                                     
        
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            # calculate odometry
            self.pub_pulsos_izq.publish(self.pulsos_IZQ)
            self.pub_pulsos_der.publish(self.pulsos_DER)
                        
            if self.pulsos_antIZQ == 0 and self.pulsos_antDER==0:
                distancia_izquierda = 0
                distancia_derecha = 0
            else:
                distancia_izquierda = (self.pulsos_IZQ - self.pulsos_antIZQ) / self.pulsos_por_metro
                distancia_derecha = (self.pulsos_DER - self.pulsos_antDER) / self.pulsos_por_metro

            self.pulsos_antIZQ = self.pulsos_IZQ
            self.pulsos_antDER = self.pulsos_DER

            # distance traveled is the average of the two wheels 
            d = ( distancia_derecha + distancia_izquierda ) / 2 

            # this approximation works (in radians) for small angles
            th = ( distancia_derecha - distancia_izquierda ) / self.dist_entre_ruedas

            # calculate velocities
            self.dx = d /elapsed
            self.dr = th /elapsed
                         
            if (d != 0):
                # calculate distance traveled in x and y
                #x = cos( th ) * d
                #y = -sin( th ) * d  
                # calculate the final position of the robot
                #self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y ) # los angulos son diferentes al anterior
                #self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )

                self.x=self.x+d*cos(self.th+th/2)
                self.y=self.y+d*sin(self.th+th/2)
                                                  
            if( th != 0):
                
                self.th = (self.th + th) %(2*pi)

            self.antes=rospy.Time.now()

        self.enc_izq=self.enc_izq+self.pulsos_IZQ
        self.enc_der=self.enc_der+self.pulsos_DER
                        
        print('\n')
        rospy.loginfo("POSICION  { X=%.2f Y=%.2f THETA=%.2f }", self.x,self.y,math.degrees(self.th))
        rospy.loginfo("enc_izq =%.2f enc_der=%.2f ", self.enc_izq,self.enc_der)

        return self.x,self.y,self.th,self.dx,self.dr
        
    def update_ale(self):
        
        now = rospy.Time.now()

        if now > self.t_next:                                                                     
        
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            # calculate odometry
            
            self.pub_pulsos_izq_ale.publish(self.pulsos_IZQ_ale)
            self.pub_pulsos_der_ale.publish(self.pulsos_DER_ale)

            distancia_izquierda_ale = (self.pulsos_IZQ_ale ) / self.pulsos_por_metro_ale
            distancia_derecha_ale = (self.pulsos_DER_ale) / self.pulsos_por_metro_ale

            # distance traveled is the average of the two wheels 
            d_ale = ( distancia_derecha_ale + distancia_izquierda_ale ) / 2 

            # this approximation works (in radians) for small angles
            th_ale = ( distancia_derecha_ale - distancia_izquierda_ale ) / self.dist_entre_ruedas_ale
            
            # calculate velocities
            self.dx_ale = d_ale /elapsed
            self.dr_ale = th_ale /elapsed

            if (d_ale != 0):
                
                self.x_ale=self.x_ale+d_ale*cos(self.th_ale+th_ale)
                self.y_ale=self.y_ale+d_ale*sin(self.th_ale+th_ale)
                            
            if( th_ale != 0):
                                
                self.th_ale = (self.th_ale + th_ale) %(2*pi)
                
            self.enc_izq_ale+=self.pulsos_IZQ_ale
            self.enc_der_ale+=self.pulsos_DER_ale
                            
            print('\n')
            rospy.loginfo("POS ALE   { X=%.2f Y=%.2f THETA=%.2f }", self.x_ale,self.y_ale,math.degrees(self.th_ale))
            rospy.loginfo("enc_izq =%.2f enc_der=%.2f ", self.enc_izq_ale,self.enc_der_ale)
                
            self.antes=rospy.Time.now()
            
        return self.x_ale,self.y_ale,self.th_ale,self.dx_ale,self.dr_ale
    
    def publish_odom(self,x,y,th,dx,dr):
            
        quaternion=tf.transformations.quaternion_from_euler(0,0,th)
        odomtf=TransformStamped()
        odomtf.header.stamp=rospy.Time.now()
        odomtf.header.frame_id="odom"
        odomtf.child_frame_id = "base_link"
        odomtf.transform.translation.x = x
        odomtf.transform.translation.y = y
        odomtf.transform.translation.z = 0
        odomtf.transform.rotation.x = quaternion[0]
        odomtf.transform.rotation.y = quaternion[1]
        odomtf.transform.rotation.z = quaternion[2]
        odomtf.transform.rotation.w = quaternion[3]
        self.odomBroadcaster.sendTransformMessage(odomtf)
        
        quaternion=tf.transformations.quaternion_from_euler(0,0,th)
        odom = Odometry()
        
        """
        odom.pose.covariance=[0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.05, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        """
        
        odom.header.stamp = odomtf.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dr
        
        #print('\n')
        #rospy.loginfo("POSICION  { X=%.2f Y=%.2f THETA=%.2f }", self.x,self.y,math.degrees(self.th))
        #rospy.loginfo("VELOCIDAD { Vx=%.2f Vth=%.2f }", (self.dx),(self.dr))#(math.degrees(self.dr)*60))
        
        self.posx=x
        self.posy=y 
        self.posth=th
        
        odomStamped=Pose2D()
        
        odomStamped.x=x
        odomStamped.y=y
        odomStamped.theta=th

        #if self.dr>10:
        #    pass
        #else:
        #    self.odomPub.publish(odom)                
        #    self.odomStamped.publish(odomStamped)
        
        self.odomPub.publish(odom)
        self.odomStamped.publish(odomStamped)
        
    #################################################
            
    def spin(self):

        while not rospy.is_shutdown():

            try:
                
                if self.i>=0:
                    
                    x,y,th,dx,dr=self.update()
                    #self.update_ale()
                    
                    self.publish_odom(x,y,th,0,0)
                
                rospy.Rate(self.frecuencia_tf).sleep()

            except Exception as e:
                rospy.loginfo("lio excepcion, %s",traceback.format_exc())
                rospy.Rate(self.frecuencia_tf).sleep()
                
if __name__ == '__main__':
    
    rospy.init_node('odometriaNODO')  
    nodoodometria=odometria()
    nodoodometria.spin()