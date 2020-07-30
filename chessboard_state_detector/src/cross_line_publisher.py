#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("img_with_cross",Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("avt_camera_img",Image,self.callback)

  def callback(self,data):
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.line(cv_image, (0,600), (1600,600), (255,0,0), thickness=2)
    cv2.line(cv_image, (800,0), (800,1200), (255,0,0), thickness=2)
    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

if __name__ == '__main__':
  rospy.init_node('cross_line_publisher', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")