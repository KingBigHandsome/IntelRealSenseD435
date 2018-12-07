#!/usr/bin/env python
import cv2
import glob
import os


class Image_proprocess():
	
	def __init__(self):
		self.src_path = '/home/wds/Desktop/My_Dataset/source/*.jpg'
		self.source_path = '/home/wds/Desktop/My_Dataset/source'
		self.destination_path = "/home/wds/Desktop/My_Dataset/destination"
	
	def rename(self):
		"""
		Rename twice to avoid image overwrite
		"""
		filelist = os.listdir(self.source_path)
		total_num = len(filelist)
		i = 1
		for item in filelist:
			index = "%06d" % i
			src = os.path.join(os.path.abspath(self.source_path), item)
			dst = os.path.join(os.path.abspath(self.source_path), index + '_.jpg')
			try:
				os.rename(src, dst)
				i = i + 1
			except:
				continue
		print 'First rename: total %d images had been renamed' % total_num
		
		filelist = os.listdir(self.source_path)
		total_num = len(filelist)
		i = 1
		for item in filelist:
			index = "%06d" % i
			src = os.path.join(os.path.abspath(self.source_path), item)
			dst = os.path.join(os.path.abspath(self.source_path), index + '.jpg')
			try:
				os.rename(src, dst)
				i = i + 1
			except:
				continue
		print 'Second rename: total %d images had been renamed' % total_num

	def image_process(self):
		
		for image in glob.glob(self.src_path):
			name_1 = os.path.basename(image).split('.')[0] + '_1.jpg'
			name_2 = os.path.basename(image).split('.')[0] + '_2.jpg'
			name_3 = os.path.basename(image).split('.')[0] + '_3.jpg'
			img = cv2.imread(image)
			self.height = img.shape[0]
			self.width = img.shape[1]
			if self.height >= 416 and self.width >= 416:
			
				img_cropped_1, img_cropped_2, img_cropped_3 = self.crop(img)
				img_resized_1, img_resized_2, img_resized_3 = self.resize(img_cropped_1, img_cropped_2, img_cropped_3)
				cv2.imwrite(os.path.join(self.destination_path, name_1), img_resized_1)
				cv2.imwrite(os.path.join(self.destination_path, name_2), img_resized_2)
				cv2.imwrite(os.path.join(self.destination_path, name_3), img_resized_3)
			else:
				img_cropped_1, img_cropped_2, img_cropped_3 = self.crop(img)
				img_resized_1, img_resized_2, img_resized_3 = self.resize(img_cropped_1, img_cropped_2, img_cropped_3)
				cv2.imwrite(os.path.join(self.destination_path, name_1), img_resized_1)
				cv2.imwrite(os.path.join(self.destination_path, name_2), img_resized_2)
				cv2.imwrite(os.path.join(self.destination_path, name_3), img_resized_3)
	
	def crop(self, image):
		center_x = self.width // 2
		center_y = self.height // 2
		if self.width >= self.height:
			cropped_image_1 = image[0:self.height, 0:self.height]
			cropped_image_2 = image[0:self.height, center_x - center_y:center_x + center_y]
			cropped_image_3 = image[0:self.height, self.width - self.height:self.width]
		else:
			cropped_image_1 = image[0:self.width, 0:self.width]
			cropped_image_2 = image[center_y - center_x:center_y + center_x, 0:self.width]
			cropped_image_3 = image[self.height-self.width:self.height, 0:self.width]
		return cropped_image_1, cropped_image_2, cropped_image_3
	
	def resize(self, image_1, image_2, image_3 ):
		resized_image_1 = cv2.resize(image_1, (416, 416), 0, 0, cv2.INTER_NEAREST)
		resized_image_2 = cv2.resize(image_2, (416, 416), 0, 0, cv2.INTER_NEAREST)
		resized_image_3 = cv2.resize(image_3, (416, 416), 0, 0, cv2.INTER_NEAREST)
		return resized_image_1, resized_image_2, resized_image_3
	
	def sum(self):
		print "total number of images im source folder is: " + str(len(os.listdir(self.source_path)))
		print "total number of images im destination is: " + str(len(os.listdir(self.destination_path)))
	
	def main(self):
		self.rename()
		self.image_process()
		self.sum()


if __name__ == '__main__':
	process = Image_proprocess()
	process.main()
