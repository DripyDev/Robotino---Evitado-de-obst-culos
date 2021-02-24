#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import numpy as np
import math as m


class Robot:
    def __init__(self):
        # ROS pub/sub
        vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")
        self._laser_sub = rospy.Subscriber("/scan", LaserScan, self.obstacleDetect, queue_size = 1)
        self.cmd_vel_pub_ = rospy.Publisher(vel_topic, Twist, queue_size = 1)

        #Subscripcion al segemet_blob
        self._segment_sub = rospy.Subscriber("/centroide", Point, self.color_follower, queue_size = 10)

        
        self._scan_count = 0
        self._max_range = 4.0
        self._first = 1
        self._bearings = []
        self._scan = []
        self.primer_angulo=120
        self.angulo_anterior=90


    #Nos pasan la informacion del centroide
    def color_follower(self,msg):
        #Creamos el mensaje del tipo Twist para dar el movimiento del robot
        cmd_vel_msg = Twist()

        #Parametros
        #Distancia de seguridad, mas cerca de esto y nos paramos
        distancia_min = 0.15

        #Creo que esta bien, x=0 es -90, x=240=90
        #angulo=(msg.x-120)*90/240*2
        #FALTA TENER EN CUENTA QUE VA A ACELERAR SIEMPRE
        #PUEDE QUE EL MAS Y MENOS SEA AL REVES
        angulo=msg.x-160

        if(msg.x == -1):
            if(self.angulo_anterior < 0):
                angulo = -100
            elif(self.angulo_anterior > 0):
                angulo = 100
            else:
                angulo = 0
        else:
            self.angulo_anterior = angulo
        #Girar a la izquierda
        #if(angulo < self.primer_angulo):
        #    angulo += 5
        #Girar a la derecha
        #elif(angulo > self.primer_angulo):
        #    angulo -= 5
        #angulo_final = angulo

        #Distancia
        #El indice correspondiente al scan cuya posicion corresponde a la x del centroide
        pos_distancia = int (257*msg.x/320)
        tmp = self._scan[128:385]
        distancia = tmp[pos_distancia]

        #Velocidad de movimiento
        velocidad=0
        #si estamos mas lejos o igual que la distancia de seguridad, nos movemos, sino no
        if(distancia >= distancia_min and msg.x != -1):
            velocidad = 0.3
        #print("msg: ", msg)
        
        cmd_vel_msg.linear.x = velocidad
        cmd_vel_msg.linear.y = 0

        cmd_vel_msg.angular.z = angulo*0.01*-1
        #print("Angulo que giramos",cmd_vel_msg.angular.z)

        self.cmd_vel_pub_.publish(cmd_vel_msg)



        
        
    def obstacleDetect(self, scan):
        self._scan = scan.ranges
        self._scan_count = len(scan.ranges)
        self._max_range = scan.range_max
        

def main():
    rospy.init_node("color_follower")
    robot = Robot()
    rospy.spin()
    
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass
#PISTAS:
#Tenemos una tabla de 180 valores.
#Modo simple y kk:
#Dividimos el barrido en tres partes, la parte 1 son los objetos de la izquierda, 2 centro y 3 derecha.
#Podriamos sacar la velocidad en funcion de la densidad de los objetos del frente.
#
#Modo mejor:
#Lo interpretamos como un histograma, toda zona con una distancia mayor a un umbral, son posibles direcciones donde no nos chocaremos
#Buscamos intervalos de distancias mayores a un umbral y nos orientamos hacia el centro de ese intervalo. Tambien podemos tener en cuenta cuanto tenemos que
#girar para ponernos en direccion segura, lo cual tendremos en cuenta para establecer la velocidad lineal del robot