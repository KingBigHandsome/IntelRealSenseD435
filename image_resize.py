import cv2
import os
import glob

src_path = "/home/wds/Desktop/src.jpg"
des_path = "/home/wds/Desktop/des.jpg"

img = cv2.imread(src_path)
cropped_image = img[0:img.shape[0], 0:img.shape[0]]
resized_image = cv2.resize(cropped_image, (416, 416), 0, 0, cv2.INTER_NEAREST)
cv2.imwrite(des_path, resized_image)


