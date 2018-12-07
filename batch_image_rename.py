# -*- coding:utf8 -*-

import os


class BatchRename():
	"""
	批量重命名文件夹中的图片文件

	"""
	def __init__(self):
		self.source_path = '/home/wds/Desktop/My_Dataset/source'
		self.renamed_path = '/home/wds/Desktop/My_Dataset/renamed'
	
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


if __name__ == '__main__':
	demo = BatchRename()
	demo.rename()
