#!/usr/bin/env python2

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math as m


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

        #Parametros
        distancia_min=1
        umbral_tamano = 60
        #Velocidad maxima
        vMax=0.4
        #Distancia a partir de la cual nos realentizamos
        dMax = 2.5
        x = range(len(self._scan))
        #Elementos de _scan cuya distancia sea mayor a distancia_min
        lista_huecos = [ j for (i,j) in zip(self._scan,x) if i > distancia_min ]
        previo = lista_huecos[0]-1
        lista = list()
        #lista de caminos consecutivos
        listaFinal = []
        maxMax = -1
        counter = 0
        #Encontrar los caminos consecutivos
        for a in lista_huecos:
            #Es consecutivo
            if((a-1) == previo):
                lista = np.append(lista,a)
            #No es consecutivo
            else:
                #Comprobar que sea la mas grande
                if(len(lista) > maxMax):
                    maxMax =  counter+1
                counter += 1
                listaFinal.append(lista)
                lista = list()
            previo = a

        if(len(lista)>0):
            counter +=1
            listaFinal.append(lista)

#        if(len(listaFinal)>1):
#            maxOfthem = listaFinal[counter-1]
#        else:
#            maxOfthem = listaFinal

        #Elegimos el camino mas grande
        tmp_max=0
        for i in range(len(listaFinal)):
            if(len(listaFinal[i])>tmp_max):
                tmp_max = len(listaFinal[i])
                maxOfthem=listaFinal[i]

        #print("Lista final: ", listaFinal)
        #print("Numero de posibles caminos: ", len(listaFinal))
        print("Longitud maxofthem: ", len(maxOfthem))
  
        #Solo usamos el camino si es mayor al umbral necesario para que el robot quepa
        if(len(maxOfthem)>=umbral_tamano):
            tmp=self._scan[int (maxOfthem[int (len(maxOfthem)/2)]) ]
            if(tmp>dMax):
                tmp=dMax
            vel=tmp*vMax/dMax
            print("Vel: ", vel)
            cmd_vel_msg.linear.x = vel
            cmd_vel_msg.linear.y = 0
            maxV = maxOfthem[0]
            minV = maxOfthem[-1]
            mediaV = int ((maxV + minV) / 2) -256
            cmd_vel_msg.angular.z = mediaV*0.025
            self.cmd_vel_pub_.publish( cmd_vel_msg )
        else:
            print("No me muevo porque no hay camino, giro")
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.angular.z = 0.5
            self.cmd_vel_pub_.publish( cmd_vel_msg )

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