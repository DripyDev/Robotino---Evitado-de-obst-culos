#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math as m
#Cosas aitor
#import keyboard

class Robot:
    def __init__(self):
        # ROS pub/sub
        vel_topic = rospy.get_param("~vel_topic", "/cmd_vel")
        self._laser_sub = rospy.Subscriber("/scan", LaserScan, self.obstacleDetect, queue_size = 1)
        self.cmd_vel_pub_ = rospy.Publisher(vel_topic, Twist, queue_size = 1)
        
        self._scan_count = 0
        self._max_range = 4.0
        self._first = 1
        self._bearings = []
        self._scan = []
        
    def obstacleDetect(self, scan):
        self._scan = scan.ranges
        self._scan_count = len(scan.ranges)
        self._max_range = scan.range_max

        long_scan=len(self._scan)

        if self._first:
            self._first = 0
            for i in range(0, self._scan_count):
                self._bearings.append(scan.angle_min + scan.angle_increment * i)
              
            print("Laser angle min:", np.rad2deg(scan.angle_min))
            print("# Laser angle max: ", np.rad2deg(scan.angle_max))
            print("# Laser angle increment: ", np.rad2deg(scan.angle_increment))
            print("# Time between mesurements [seconds]: ", scan.time_increment)
            print("# Time between scans [seconds]: ", scan.scan_time)
            print("# Minimum range value: ", scan.range_min)
            print("# Maximum range value: ", scan.range_max)
            resolution = (scan.angle_max - scan.angle_min)/len(scan.ranges)
            print("# Resolution: %f\n"%(np.rad2deg(resolution)))
            

        # EXAMPLE: get min/max values of each scan
        rminidx  = np.argmin(scan.ranges)
        rmaxidx  = np.argmax(scan.ranges)
        rmax = np.max(scan.ranges)
        rmin = np.min(scan.ranges)
        cmd_vel_msg = Twist()

        #Procesamos la lista _scan en un array


        if(self._scan[512]< 0.15):
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = -0.1
            cmd_vel_msg.angular.z = m.radians(0)
            self.cmd_vel_pub_.publish( cmd_vel_msg )
            print("Algo verca de mi, me muevo en -y")
        if(self._scan[0]< 0.15):
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0.1
            cmd_vel_msg.angular.z = m.radians(0)
            self.cmd_vel_pub_.publish( cmd_vel_msg )
            print("Algo verca de mi, me muevo en +y")
        #Si presionamos espacio, paramos
        #parar=False
        #if keyboard.is_pressed("space"):
        #    parar = not parar

        #Parametros
        distancia_min=0.3
        umbral_tamano = 20
        
        #Lista de listas con los posibles caminos
        lista_huecos=[[]]
        #Recorremos la lista de angulos
        for i in range(len(self._scan)):
            lista_aux=[]
            #Posible lista
            if(self._scan[i]>distancia_min):
                #Guardamos el indice del angulo cuya distancia es segura
                lista_aux.append(i)
            #Se rompe la continuidad de la lista
            else:
                #Comprobamos que la lista sea mayor al umbral para que quepa el robot
                if(len(lista_aux)>=umbral_tamano):
                    #La lista auxiliar es valida, la guardamos
                    lista_huecos.append(lista_aux)
                #Reseteamos la lista
                lista_aux=[]
        #Con la lista de posibles huecos, encontramos el mas grande en extension
        max=0
        camino_final=[]
        #Elegimos el camino mas grande
        for i in range(len(lista_huecos)):
            if(len(lista_huecos[i])>max):
                max=len(lista_huecos[i])
                camino_final=lista_huecos[i]
        #Con el camino mas grande, nos movemos al centro de dicho camino
        if(len(camino_final>0)):
            cmd_vel_msg.linear.x = 0.2
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.angular.z = camino_final[int (len(camino_final)/2) ]*scan.angle_increment
            self.cmd_vel_pub_.publish( cmd_vel_msg )
        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub_.publish( cmd_vel_msg )






        print("Distancias enfrente: ",vecDistancias[8],vecDistancias[9],vecDistancias[10])

        #print("Length _scan: ",long_scan)
        #print("Length _bearings: ",len(self._bearings))
        #print("Valor _scan: ",self._scan[(long_scan/2-3):(long_scan/2+3)])
        #print("Valor _bearings: ",self._bearings[87:92])
        #print("Range min: %.2f (%d) Range max: %.2f (%d)"%(rmin, rminidx, rmax, rmaxidx))

def main():
    rospy.init_node("robot_wander")
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