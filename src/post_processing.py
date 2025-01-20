#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

# Node name
NODE_NAME = 'post_processing_node'


pub = rospy.Publisher('post_processed_image', Image, queue_size=10)
bridge = CvBridge()

def callback(msg):
  bin = bridge.imgmsg_to_cv2(msg, 'mono8')
  output = bin
  output = cv2.erode(output, np.ones((3,3),np.uint8), iterations = 1)
  output = cv2.dilate(output, np.ones((3,3),np.uint8), iterations = 1)
  
  output = cv2.dilate(output, np.ones((3,3),np.uint8), iterations = 2)
  output = cv2.erode(output, np.ones((3,3),np.uint8), iterations = 2)
  
  img_msg = bridge.cv2_to_imgmsg(output, 'mono8')
  pub.publish(img_msg)

sub = rospy.Subscriber('/hsv_filter/image_mask', Image, callback)
rospy.init_node('post_processing_node')

rospy.spin()

if __name__=="__main__":
  rospy.init_node(NODE_NAME, anonymous=False)
  rospy.spin()