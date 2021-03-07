#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String


# Map class: size, scale and bitmap
class Map:   
   def __init__(self, rows=1000, cols=1000, scale=20):
      ''' constructor function
        '''
      # Map properties
      self.maxXsize = cols
      self.maxYsize = rows
      self.scale = scale
      self.ox = cols/2
      self.oy = rows/2
      
      self.map = np.zeros((rows, cols, 3), np.uint8)

      #Camara adicional
      self.bridge = CvBridge()
      topic_name = rospy.get_param("image_topic", "/image_raw")
      self.sub = rospy.Subscriber(topic_name, Image, self.callback)

      
   def setObstacle(self, xpos, ypos, color):
      i = int(self.ox + xpos * self.scale)
      j = int(self.oy + ypos * self.scale)
      print("i, j: ", i, j)
      if i < 0 or i > self.maxXsize:
         print("Error. X size exceeded")
         return -1
      if j < 0 or j > self.maxYsize:
         print("Error. Y size exceeded")
         return -1
      self.map[i,j] = color
      return 1

   def callback(self, msg):
      try:
         #Imagen raw de la camara en sistema bgr8
         cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
          
         cv2.imshow("Blob segmentation", cv_img)
         cv2.waitKey(1)

      except CvBridgeError as exc:
         print(traceback.format.exc())

class Robot:
   def __init__(self):
      self._laser_sub = rospy.Subscriber("/scan", LaserScan, self.laserProcessing, queue_size = 1)
      self._odom_sub = rospy.Subscriber("/odom", Odometry, self.processOdometry, queue_size = 1)

      # Robot pose information
      self._rx = 0
      self._ry = 0
      self._rtheta = 0
      self._scan = []
      self._scan_count = 0
      self._max_range = 4.0
      self._first = 1
      self.bearings = []
    
   def processOdometry(self, odom_msg):
      self._rx = odom_msg.pose.pose.position.x
      self._ry = odom_msg.pose.pose.position.y
      quat = odom_msg.pose.pose.orientation
      _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
      self._rtheta = yaw
      
   def laserProcessing(self, scan_msg):
      self._scan = scan_msg.ranges
      self._scan_count = len(scan_msg.ranges)
      self._max_range = scan_msg.range_max
      if self._first:
          self._first = 0
          for i in range(0, self._scan_count):
              self.bearings.append(scan_msg.angle_min + scan_msg.angle_increment * i)



def main():
    color0 = (0, 0, 255)
    color1 = (255, 0, 0)
    rospy.init_node("laser_mapping")
    #El laser lee la informacion mas rapido que la odometria, asi que hay menos ruido en el mapa si reducimos el ratio de lectura del laser
    r = rospy.Rate(10)
    mapa = Map()
    robot = Robot()
    while not rospy.is_shutdown():
       mapa.setObstacle(robot._rx, robot._ry, color0)
       for i in range(0, robot._scan_count):
          if robot._scan[i] < robot._max_range:
             # TODO
             lx = robot._rx + robot._scan[i] * math.cos(robot._rtheta + robot.bearings[i])
             ly = robot._ry + robot._scan[i] * math.sin(robot._rtheta + robot.bearings[i])
             # End TODO
             mapa.setObstacle(lx, ly, color1)
       cv2.imshow("Mapa", mapa.map)
       cv2.waitKey(1)
       r.sleep()
    
if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass
