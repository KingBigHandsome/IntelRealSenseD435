#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('realsense2_camera')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Image_converter:
	
	def __init__(self):
		# self.image_pub = rospy.Publisher("image_topic_2", Image)
		
		self.bridge = CvBridge()
		self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_callback)
	
	def color_callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		
		(rows, cols, channels) = cv_image.shape
		if cols > 60 and rows > 60:
			cv2.circle(cv_image, (200, 200), 100, (0, 0, 0), 2)
		
		cv2.imshow("Color Image window", cv_image)
		cv2.waitKey(3)
		
		try:
			pass
			# self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)


def main(args):
	ic = Image_converter()
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
