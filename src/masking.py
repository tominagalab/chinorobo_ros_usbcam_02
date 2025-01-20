#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

# Node name
NODE_NAME = 'masking_node'

bridge = CvBridge()
pub = rospy.Publisher('/'+NODE_NAME+'/output_image', Image, queue_size=5)

raw = None
mask = None

def callback(msg_raw, msg_mask):
  rospy.loginfo('callback')
  
  raw = bridge.imgmsg_to_cv2(msg_raw)
  mask = bridge.imgmsg_to_cv2(msg_mask)
  
  output = cv2.bitwise_and(raw, raw, mask=mask)
  
  msg_ouptut = bridge.cv2_to_imgmsg(output, 'bgr8')
  
  pub.Publish(msg_output)
  return

def callback_raw(msg):
  # rospy.loginfo('callback raw image')
  
  global raw
  raw = bridge.imgmsg_to_cv2(msg)

def callback_mask(msg):
  # rospy.loginfo('callback mask')

  global raw
  global mask
  mask = bridge.imgmsg_to_cv2(msg)
  
  output = cv2.bitwise_and(raw, raw, mask=mask)
  msg_output = bridge.cv2_to_imgmsg(output, 'bgr8')
  pub.publish(msg_output)


rospy.init_node(NODE_NAME, anonymous=False)

# sub_raw = message_filters.Subscriber('/raw', Image)
# sub_mask = message_filters.Subscriber('/mask', Image)

# fps = 1.
# delay = 1 / fps * 0.5

# rospy.loginfo('test')
# mf = message_filters.ApproximateTimeSynchronizer([sub_raw, sub_mask], 10, 0.5, allow_headerless=True)
# mf.registerCallback(callback)

sub_raw = rospy.Subscriber('raw', Image, callback_raw)
sub_mask = rospy.Subscriber('mask', Image, callback_mask)

rospy.spin()