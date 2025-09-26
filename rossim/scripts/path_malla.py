# -*- coding: utf-8 -*-  
#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import tf
from threading import Lock

class trayectoriaFollower:
    def __init__(self):
        
        # Publicador de comandos de velocidad
        
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Subscripciones a odometría y trayectoria
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('trajectory', Path, self.path_callback)
        
        # Estado actual del robot y trayectoria
        
        self.pose_actual = None
        self.trayectoria = []
        self.nueva_trayectoria_recibida = False
        self.trayectoria_lock = Lock()

        self.rate = rospy.Rate(30)  # 30 Hz

        # Parámetros del controlador
        self.pose_alcanzada_threshold = 0.1
        self.k_linear = 0.5  # ajustar suavidad
        self.k_angular = 1.0

        # Límites de velocidad
        self.max_linear_speed = 0.1  # m/s
        self.max_angular_speed = 0.2  # rad/s

    def odom_callback(self, msg):
        self.pose_actual = msg.pose.pose

    def path_callback(self, msg):
        rospy.loginfo("Recibida nueva trayectoria con {} puntos".format(len(msg.poses)))
        with self.trayectoria_lock:
            self.trayectoria = msg.poses
            self.nueva_trayectoria_recibida = True

    def follow_trayectoria(self):
        
        # Bucle principal para seguir los puntos de la trayectoria
        
        while not rospy.is_shutdown():
            if self.pose_actual is None or not self.trayectoria:
                rospy.loginfo_throttle(5, "Esperando odometria y trayectoria...")
                self.rate.sleep()
                continue

            marcador_index = 0

            # Iteracion sobre los puntos
            
            while marcador_index < len(self.trayectoria) and not rospy.is_shutdown():
                with self.trayectoria_lock:
                    if self.nueva_trayectoria_recibida:
                        rospy.loginfo("Nueva trayectoria detectada, reiniciando seguimiento...")
                        marcador_index = 0
                        self.nueva_trayectoria_recibida = False
                        continue

                    marcador = self.trayectoria[marcador_index]

                objetivo = marcador.pose.position
                alcanzado = False
                rospy.loginfo("Avanzando hacia el marcador: x={:.2f}, y={:.2f}".format(objetivo.x, objetivo.y))
                
                # Bucle de control hacia el punto actual

                while not alcanzado and not rospy.is_shutdown():
                    x_actual = self.pose_actual.position.x
                    y_actual = self.pose_actual.position.y
                    dx = objetivo.x - x_actual
                    dy = objetivo.y - y_actual
                    distancia = math.sqrt(dx**2 + dy**2)

                    desired_yaw = math.atan2(dy, dx)
                    current_yaw = self.get_yaw_from_pose(self.pose_actual)
                    error_yaw = self.normalizar_angulo(desired_yaw - current_yaw)

                    if distancia < self.pose_alcanzada_threshold:
                        alcanzado = True
                        break

                    twist = Twist()

                    # Si el error angular es grande, solo girar
                    if abs(error_yaw) > math.radians(30):
                        twist.linear.x = 0.0
                        twist.angular.z = max(-self.max_angular_speed,
                                            min(self.k_angular * error_yaw, self.max_angular_speed))
                    # Si el error angular es pequeño, combinar giro y avance
                    else:
                        twist.linear.x = min(self.k_linear * distancia, self.max_linear_speed)
                        twist.angular.z = max(-self.max_angular_speed,
                                            min(self.k_angular * error_yaw, self.max_angular_speed))

                    self.cmd_vel_pub.publish(twist)
                    self.rate.sleep()

                    # Revisar si hay nueva trayectoria durante movimiento
                    with self.trayectoria_lock:
                        if self.nueva_trayectoria_recibida:
                            rospy.loginfo("Nueva trayectoria detectada, interrumpiendo marcador actual.")
                            break

                self.cmd_vel_pub.publish(Twist())
                #rospy.sleep(0.3)
                marcador_index += 1

            rospy.loginfo("Trayectoria completada.")
            with self.trayectoria_lock:
                self.trayectoria = []

    def get_yaw_from_pose(self, pose):
        orientation = pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler[2]

    def normalizar_angulo(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == '__main__':
    rospy.init_node('trayectoria_follower')
    follower = trayectoriaFollower()
    try:
        follower.follow_trayectoria()
    except rospy.ROSInterruptException:
        pass