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
import numpy as np
import time
# global color_image, depth_colormap
color_image = np.zeros((640, 480, 3))
depth_colormap = np.zeros((640, 480, 3))


def color_callback(data):
	
	global color_image
	
	bridge = CvBridge()
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		color_image = np.asanyarray(cv_image)
	except CvBridgeError as e:
		print(e)
	
	(rows, cols, channels) = color_image.shape
	if cols > 60 and rows > 60:
		cv2.circle(color_image, (200, 200), 100, (0, 0, 0), 2)
	

def depth_callback(data):
	
	global depth_colormap
	
	bridge = CvBridge()
	
	print("Info of Depth Image:\nimage header is %s\nimage height is %s\nimage width is %s\nimage encoding is %s\n"
	      % (data.header, data.height, data.width, data.encoding))
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "16UC1")
		depth_image = np.asanyarray(cv_image)
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
	except CvBridgeError as e:
		print(e)
	
	(rows, cols, channels) = depth_colormap.shape
	if cols > 60 and rows > 60:
		cv2.circle(depth_colormap, (200, 200), 100, (255, 0, 255), 2)
	
	
		

def main(args):

	rospy.init_node('image_converter', anonymous=True)
	color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
	depth_image_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
	
	time.sleep(2)
	
	while True:
		image = np.hstack((color_image, depth_colormap))
		
		cv2.imshow("Image window", image)
		key = cv2.waitKey(1)
		if key & 0xFF == ord('q') or key == 27:
			cv2.destroyAllWindows()
		try:
			rospy.spin()
		except KeyboardInterrupt:
			print("Shutting down")
		cv2.destroyAllWindows()
		
	
if __name__ == '__main__':
	main(sys.argv)
