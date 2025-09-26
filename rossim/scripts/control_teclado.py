#!/usr/bin/env python

import rospy,ros
from geometry_msgs.msg import *
from sensor_msgs.msg import *

import threading,sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

anything else : stop

+/- : aumentar/disminuir velocidades maximas en 10% (min 10%)
r/v : aumentar/disminuir solo velocidad linear en 10% (min 10%)
t/b : aumentar/disminuir solo velocidad angular en 10% (min 10%)

CTRL-C to quit
"""

print(msg)

rospy.init_node('CONTROL_TECLADO_NODO')

class Variables_(object):
    
    multi=1
    key=''
    vel=Twist()
    vel_lin=1
    vel_ang=1
    
Variables=Variables_()

class Publicaciones_():
    pub_vel=rospy.Publisher('cmd_vel',Twist,queue_size=1)

pubs=Publicaciones_()

class Funciones_(object):
    
    def getKey(self,key_timeout):
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

Funciones=Funciones_()

class Control_nodo(object):
    
    def __init__(self) :
        
        pass
    
    def tecla_vel(self,tecla):
                            
        if tecla=='w':
            
            Variables.vel=Twist()
            
            Variables.vel.linear.x=1.0*Variables.vel_lin/10.0
            
        elif tecla=='x':
            
            Variables.vel=Twist()
                        
            Variables.vel.linear.x=-1.0*Variables.vel_lin/10.0
            
        elif tecla=='a':
            
            Variables.vel=Twist()
                        
            Variables.vel.angular.z=1.0*Variables.vel_ang/10.0
            
        elif tecla=='d':
            
            Variables.vel=Twist()
                                    
            Variables.vel.angular.z=-1.0*Variables.vel_ang/10.0
            
        elif tecla=='s':
                                    
            Variables.vel=Twist()
            
            Variables.multi=0
        
        elif tecla=='q':
            
            Variables.vel=Twist()
                                    
            Variables.vel.linear.x=1.0*Variables.vel_lin/10.0
            Variables.vel.angular.z=1.0*Variables.vel_ang/10.0
                    
        elif tecla=='e':
            
            Variables.vel=Twist()
                                    
            Variables.vel.linear.x=1.0*Variables.vel_lin/10.0
            Variables.vel.angular.z=-1.0*Variables.vel_ang/10.0
                    
        elif tecla=='z':
            
            Variables.vel=Twist()
                                    
            Variables.vel.linear.x=-1.0*Variables.vel_lin/10.0
            Variables.vel.angular.z=1.0*Variables.vel_ang/10.0
                    
        elif tecla=='c':
            
            Variables.vel=Twist()
                                    
            Variables.vel.linear.x=-1.0*Variables.vel_lin/10.0
            Variables.vel.angular.z=-1.0*Variables.vel_ang/10.0
                        
        #elif tecla=='+':
        #    
        #    Variables.vel_lin+=1
        #    Variables.vel_ang+=1
        #    
        #elif tecla=='-':
        #    
        #    Variables.vel_ang-=1
        #    Variables.vel_lin-=1
            
        elif tecla=='r':
            
            Variables.vel_lin+=1
            
        elif tecla=='v':
            
            Variables.vel_lin-=1
        
        elif tecla=='t':
            
            Variables.vel_ang+=1
            
        elif tecla=='b':
            
            Variables.vel_ang-=1

        else:
            
            Variables.vel=Twist()
        
        pubs.pub_vel.publish(Variables.vel)
        #return self.vel
      
    def run(self):
                
        while not rospy.is_shutdown():
            
            rospy.spin()
 
control=Control_nodo()

def run_keyboard():

    key_timeout = 0.0
    
    if key_timeout == 0.0:
        key_timeout = None
    try:

        while(1):
                    
            Variables.key = Funciones.getKey(key_timeout)
            
            if (Variables.key == '\x03'):
                break
            
            #print('hola',Variables.key)
            control.tecla_vel(Variables.key)
                        
    except Exception as e:
        print(e)

    finally:

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings) 
           
if __name__ == "__main__":
    
    control=Control_nodo()
    
    thread=threading.Thread(target=run_keyboard)
    thread.setDaemon(True)
    thread.start()

    control.run()    
    