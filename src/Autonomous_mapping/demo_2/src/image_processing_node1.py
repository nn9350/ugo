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
    self.colour_frame = None
    self.depth_frame = None
    self.depth_image = 0
    self.num_colour_images = 0
    self.num_depth_images = 0
    self.K = None
    self.transform_cam_to_world = None
    self.subscriber_camera_info = rospy.Subscriber('camera/rgb/camera_info',CameraInfo, self.callback_camera_info)
    self.subscriber_odometry = rospy.Subscriber('/odom',Odometry, self.callback_odometry)
    self.subscriber_colour = rospy.Subscriber('/camera/rgb/image_raw/compressed',CompressedImage, self.callback_colour)
    self.subscriber_depth = rospy.Subscriber('/camera/depth/image_raw',Image, self.callback_depth)
    self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    self.marker_array = MarkerArray()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.loop()
      r.sleep()
  
  def callback_colour(self,colour_image):
    rospy.loginfo('[Image Processing] callback_colour')
    np_array = np.fromstring(colour_image.data, np.uint8)
    self.colour_frame = cv2.imdecode(np_array, cv2.IMREAD_COLOR)
    blurred = cv2.GaussianBlur(self.colour_frame, (11,11),0)
    hsv =cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    colour_lower_r = (0, 137, 0)
    colour_upper_r = (7, 255, 255)
    colour_lower_b = (98, 116, 59)
    colour_upper_b = (137, 253, 255)
    colour_lower_y = (25, 100, 0)
    colour_upper_y = (39, 255, 255)
    colour_lower_g = (50, 106, 0)
    colour_upper_g = (74, 255, 255)
    mask_r = cv2.inRange(hsv, colour_lower_r, colour_upper_r)
    mask_b = cv2.inRange(hsv, colour_lower_b, colour_upper_b)
    mask_y = cv2.inRange(hsv, colour_lower_y, colour_upper_y)
    mask_g = cv2.inRange(hsv, colour_lower_g, colour_upper_g)
    mask_r = cv2.erode(mask_r, None, iterations=2)
    mask_r = cv2.dilate(mask_r, None, iterations=2)
    mask_b = cv2.erode(mask_b, None, iterations=2)
    mask_b = cv2.dilate(mask_b, None, iterations=2)
    mask_y = cv2.erode(mask_y, None, iterations=2)
    mask_y = cv2.dilate(mask_y, None, iterations=2)
    mask_g = cv2.erode(mask_g, None, iterations=2)
    mask_g = cv2.dilate(mask_g, None, iterations=2)
    merged = cv2.addWeighted(mask_r, 1, mask_b, 1, 0)
    merged = cv2.addWeighted(merged, 1, mask_y, 1, 0)
    self.merged = cv2.addWeighted(merged, 1, mask_g, 1, 0)
    p3d_w_r = self.find_3d_point_world(mask_r)
    #p3d_w_b = self.find_3d_point_world(mask_b)
    #p3d_w_y = self.find_3d_point_world(mask_y)
    #p3d_w_g = self.find_3d_point_world(mask_g)
    if p3d_w_r is not None:
      marker = self.create_marker(p3d_w_r[0],p3d_w_r[1])
      self.marker_pub.publish(marker)
    
  def callback_depth(self,depth_image):
   rospy.loginfo('[Image Processing] callback_depth')
   self.depth_frame = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="passthrough")

   # Replace NaN values with 0
   self.depth_frame = np.nan_to_num(self.depth_frame, nan=0.0)
   if np.isnan(self.depth_frame).any():
     print('It still has Nan values')
  
  def loop(self):   
     if self.colour_frame is not None and self.colour_frame.shape: 
       cv2.imshow('Colour Image', self.colour_frame)
       cv2.imshow('Masked Image',self.merged)
       resp = cv2.waitKey(80)
       if resp == ord('c'):
         rospy.loginfo('[Image Processing] Saving colour')
         cv2.imwrite('colour_{}.png'.format(self.num_colour_images),self.colour_frame)
         self.num_colour_images += 1
     if self.depth_frame is not None:
       self.depth_image = cv2.normalize(self.depth_frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)
       cv2.imshow('Depth Image',self.depth_frame)
       resp = cv2.waitKey(80)
       if resp == ord('d'):
          rospy.loginfo('[Image Processing] Saving depth')
          cv2.imwrite('depth_{}.png'.format(self.num_depth_images),self.depth_image)
          self.num_depth_images += 1
  
  def create_marker(self,x, y):
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.56
    marker.color.g = 0.64
    marker.color.b = 0.1
    return marker

  
  def callback_camera_info(self, camera_info):
    self.K = np.array(camera_info.K).reshape([3,3])
  
  def callback_odometry(self, odometry):
    self.transform_cam_to_world = trans.msg_to_se3(odometry.pose.pose)
  
  def find_3d_point_world(self, mask):
    contours = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) ==0:
      return None
    largest_contour = max(contours, key =cv2.contourArea)
    x,y,w,h = cv2.boundingRect(largest_contour)
    x = x + w/2
    y = y - h/2

    if self.K is None or self.transform_cam_to_world is None or self.depth_frame is None or self.K.shape != (3, 3) or self.transform_cam_to_world.shape != (4, 4) or self.depth_frame.size == 0:
      print('One or more required variables is not set properly.')
      return
    depth = self.depth_frame[y, x]
    p_h = np.array([[x],[y],[1]])
    p3d = depth*np.matmul(np.linalg.inv(self.K),p_h)
    p3d_h = np.array([[p3d[2][0]], [-p3d[0][0]],[-p3d[1][0]],[1]])
    p3d_w_h = np.matmul(self.transform_cam_to_world, p3d_h)
    p3d_w = np.array([[p3d_w_h[0][0]/p3d_w_h[3][0]],[p3d_w_h[1][0]/p3d_w_h[3][0]],[p3d_w_h[2][0]/p3d_w_h[3][0]]])
    return p3d_w
    colour_cv_frame = cv2.rectangle(largest_contour, (x,y),(x+w, y+h), (0, 0, 255),2)



if __name__ == '__main__':
  print("Starting ROS Image Processing module")
  rospy.init_node('image_processing_node', anonymous=True, log_level=rospy.DEBUG)
  ipn = image_processing_node('image_processing_node')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down ROS Image Processing module")
