#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import imutils
import transformations as trans
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker

class image_processing_node:
  def __init__(self, name):
    self.bridge = CvBridge()
    self.depth_frame = None
    self.depth_image = 0
    self.num_depth_images = 0
    self.subscriber_depth = rospy.Subscriber('/camera/depth/image_raw',Image, self.callback_depth)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.loop()
      r.sleep()
  
  
    
  def callback_depth(self,depth_image):
   rospy.loginfo('[Image Processing] callback_depth')
   self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

  def loop(self):   
     if self.depth_frame is not None:
       self.depth_image = cv2.normalize(self.depth_frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
       cv2.imshow('Depth Image',self.depth_frame)
       resp = cv2.waitKey(80)
       if resp == ord('d'):
          rospy.loginfo('[Image Processing] Saving depth')
          cv2.imwrite('depth_{}.png'.format(self.num_depth_images),self.depth_image)
          self.num_depth_images += 1
 
if __name__ == '__main__':
  print("Starting ROS Image Processing module")
  rospy.init_node('image_processing_node', anonymous=True, log_level=rospy.DEBUG)
  ipn = image_processing_node('image_processing_node')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down ROS Image Processing module")
