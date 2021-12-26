#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    #self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.callback)

  def callback(self,data):
    cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    
    (rows,cols) = cv_image.shape
    # rospy.loginfo("Shape of the imagee: {}".format((rows,cols)))
    # rospy.loginfo("Maximum image values: {}".format(np.max(cv_image)))

    centre_pixel_depth = cv_image[int(rows/2.0),int(cols/2.0)]
    rospy.loginfo("Distance of the centre pixel from the camera {}".format(centre_pixel_depth))

    #cv_image = cv2.circle(cv_image, (int(rows/2.0),int(cols/2.0)), 100, 255,5)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    # try:
    #   cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
    # except CvBridgeError as e:
    #   print(e)

    # #(rows,cols,channels) = cv_image.shape
    # print("Image shape:")
    # print("Rows: {} Columns: {} Channels: {}".format(rows,cols,channels))
    
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (int(rows/2.0),int(cols/2.0)), 10, 255)

    # cv2.imshow("Image window", cv_image)
    # cv2.waitKey(3)

    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
