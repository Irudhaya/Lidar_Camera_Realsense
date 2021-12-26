#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def depth_image_callback(depth_image):

	centre_pixel_x = depth_image.width/2.0
	centre_pixel_y = depth_image.height/2.0

	
	# centre_pixel_depth = depth_image.data[int(centre_pixel_x),int(centre_pixel_y)]

	cv_image = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
	
	rospy.loginfo("Shape of the image: ",cv_image.shape)
	exit()
	#(rows,cols) = cv_image.shape

	cv2.circle(cv_image, (int(rows/2.0),int(cols/2.0)), 10, 255)

	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)

	rospy.loginfo("Distance of the centre pixel from the camera {}".format(centre_pixel_depth))



rospy.init_node('depth_image')
rospy.loginfo("Subscribing to the topic of the depth image")
rospy.Subscriber("/camera/depth/image_rect_raw",Image,depth_image_callback)
rospy.spin()


