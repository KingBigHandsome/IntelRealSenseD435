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
import signal

# color_image = np.zeros((640, 480, 3))
# depth_colormap = np.zeros((640, 480, 3))
color_enable = False
depth_enable = False


def signal_handler(signal, frame):
	print('You pressed Ctrl+C!')
	sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def color_callback(data):
	global color_enable, color_image
	bridge = CvBridge()
	
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		color_image = np.asanyarray(cv_image)
	except CvBridgeError as e:
		print(e)
	
	(rows, cols, channels) = color_image.shape
	if cols > 60 and rows > 60:
		cv2.circle(color_image, (200, 200), 100, (0, 0, 0), 2)
	
	color_enable = True
	print('color images are ready!')


def depth_callback(data):
	global depth_enable, depth_colormap
	bridge = CvBridge()
	
	try:
		cv_image = bridge.imgmsg_to_cv2(data, "16UC1")
		depth_image = np.asanyarray(cv_image)
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
	except CvBridgeError as e:
		print(e)
	
	(rows, cols, channels) = depth_colormap.shape
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
	
	depth_enable = True
	print('depth images are ready!')
	
	combination()


def combination():
	global color_enable, depth_enable
	
	if color_enable and depth_enable:
		color_enable = False
		depth_enable = False
		print('Both color and depth images are ready')
		image = np.hstack((color_image, depth_colormap))
		cv2.imshow("Image window", image)
		for i in range(100):
			path = 'test'+str(i)+'.jpg'
			cv2.imwrite(path, image)		
		cv2.waitKey(3)


def main():
	rospy.init_node('image_converter', anonymous=True)
	rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
	rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main()
