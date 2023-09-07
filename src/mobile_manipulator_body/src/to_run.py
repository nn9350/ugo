#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
def callback(data):
  pub=rospy.Publisher('/robot_base_velocity_controller/cmd_vel',Twist,queue_size=10)
  pub.publish(data)
  print('hi')
rospy.init_node('changing_topic' ,anonymous=True)
while 1:
  rospy.Subscriber('/cmd_vel',Twist,callback)
  rate=rospy.Rate(10)
