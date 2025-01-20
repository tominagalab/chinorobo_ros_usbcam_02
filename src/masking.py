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

def callback(msg_raw, msg_mask):
  rospy.loginfo('callback')
  raw = bridge.imgmsg_to_cv2(msg_raw)
  mask = bridge.imgmsg_to_cv2(msg_mask)
  
  output = cv2.bitwise_and(raw, raw, mask=mask)
  
  msg_ouptut = bridge.cv2_to_imgmsg(output, 'bgr8')
  
  pub.Publish(msg_output)
  
if __name__=="__main__":
  rospy.init_node(NODE_NAME, anonymous=False)
  
  sub_raw = message_filters.Subscriber('/'+NODE_NAME+'/raw_image', Image)
  sub_mask = message_filters.Subscriber('/'+NODE_NAME+'/mask_image', Image)
  
  
  fps = 100.
  delay = 1 / fps * 0.5
  
  mf = message_filters.ApproximateTimeSynchronizer([sub_raw, sub_mask], 10, delay)
  mf.registerCallback(callback)
  
  rospy.spin()