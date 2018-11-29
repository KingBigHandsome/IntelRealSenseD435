## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: ", depth_scale)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)

try:
	while True:

		# Wait for a coherent pair of frames: depth and color
		frames = pipeline.wait_for_frames()
		# frames.get_depth_frame() is a 640x360 depth image
		
		# Align the depth frame to color frame
		aligned_frames = align.process(frames)
		
		# Get aligned frames
		aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
		color_frame = aligned_frames.get_color_frame()
		
		if not aligned_depth_frame or not color_frame:
			continue

		# Convert images to numpy arrays
		depth_image = np.asanyarray(aligned_depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		# Apply colormap on depth image (image must be converted to 8-bit per pixel first)
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

		# Stack both images horizontally
		images = np.hstack((color_image, depth_colormap))
		
		# Draw a user-defined rectangle over the image(image name, left-top point, right-bottom point, line color, line width)
		cv2.rectangle(images, (100, 100), (400, 400), (255, 0, 0), 2)
		
		# Show images
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', images)
		key = cv2.waitKey(1)
		if key & 0xFF == ord('q') or key == 27:
			print(depth_image*depth_scale)
			cv2.destroyAllWindows()
			break
			
finally:

	# Stop streaming
	pipeline.stop()
