#!/usr/bin/env python
from __future__ import print_function

import roslib

roslib.load_manifest('realsense2_camera')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Image_converter:
	
	def __init__(self):
		# self.image_pub = rospy.Publisher("image_topic_2", Image)
		
		self.bridge = CvBridge()
		self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
	
	def depth_callback(self, data):
		
		print(
			"Info of Depth Image:\nimage.header is %s\nimage.height is %s\nimage.width is %s\nimage.encoding is %s\nimage.step is %s\n"
			% (data.header, data.height, data.width, data.encoding, data.step))
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
			depth_image = np.asanyarray(cv_image)
			depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
		except CvBridgeError as e:
			print(e)
		
		(rows, cols, channels) = depth_colormap.shape
		# print (rows, cols, channels)
		if cols > 60 and rows > 60:
			cv2.circle(depth_colormap, (200, 200), 100, (255, 0, 255), 2)
		
		distance = depth_image[200, 200] * 0.001
		print('The distance date of the circle center is %6.3f' % distance)

		cv2.putText(depth_colormap,
			    'The distance date of the circle center is' + str(distance),
			    (50, 400),
			    cv2.FONT_HERSHEY_COMPLEX,
			    0.7,
			    (0, 0, 255),
			    1)
		cv2.imshow("Depth Image window", depth_colormap)
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
