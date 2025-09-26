#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Bool

def main():
    rospy.init_node('coppelia_sim_synchronizer')

    # Espera a que los servicios de CoppeliaSim estén disponibles
    rospy.wait_for_service('/vrep/simRosPauseSimulation')
    rospy.wait_for_service('/vrep/simRosStartSimulation')

    pause_simulation = rospy.ServiceProxy('/vrep/simRosPauseSimulation', Empty)
    start_simulation = rospy.ServiceProxy('/vrep/simRosStartSimulation', Empty)

    # Publicador para enviar la señal de paso
    step_publisher = rospy.Publisher('/stepTrigger', Bool, queue_size=1)

    # Pausa y luego inicia la simulación en modo síncrono
    pause_simulation()
    start_simulation()

    rate = rospy.Rate(10)  # Frecuencia de sincronización en Hz

    while not rospy.is_shutdown():
        step_publisher.publish(Bool(data=True))  # Envía una señal para avanzar un paso en la simulación
        rate.sleep()

if __name__ == '__main__':
    main()
