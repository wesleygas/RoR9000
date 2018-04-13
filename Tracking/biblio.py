# -*- coding:utf-8 -*-
import cv2
import numpy as np

class objetoo():

	def __init__(self):
		self.temobjeto = False
		self.tembackground = False
		self.setup = False
		self.objeto = 0
		self.background = 0
		self.des1 = 0
		self.kp1 = 0
		self.flann = 0
		self.guds = []

	def crop_object(self,BG,objonBG):
		img_get = cv2.subtract(BG,objonBG)
		(idx,idy) = np.where((img_get[:,:,0] > 30) & (img_get[:,:,1] > 30) & (img_get[:,:,2] > 30))
		if len(idx) > 0 and len(idy) > 0:
			(topx,topy) = (np.min(idx),np.min(idy))
			(botx,boty) = (np.max(idx),np.max(idy))
			difx = round((topx - botx)*0.1)
			dify = round((topy - boty)*0.1)
			topx -= difx
			botx += difx
			topy -= dify
			boty += dify
			# objonBG[(img_get[:,:,0] == 0) & (img_get[:,:,1] == 0) & (img_get[:,:,2] == 0)] = 0
			return objonBG[topx:botx + 1, topy:boty + 1]
		return objonBG

	def object_coordinates(self, kp2, matches):

		meanx = 0
		meany = 0
		if len(matches) > 0:
			for mat in matches:

				b = kp2[mat.trainIdx].pt
				meanx += b[0]
				meany += b[1]

			meanx /= len(matches)
			meany /= len(matches)
		return (meanx,meany)

	def findobject(self,img2):
		# print('chamou')
		if self.setup:

			# print('entrou')

			kp2, des2 = self.sift.detectAndCompute(img2,None)

			# print('detectou')

			matches = self.flann.knnMatch(self.des1,des2,k=2)

			# print('matchou')

			good = []
			for m,n in matches:
				if m.distance < 0.7*n.distance:
					good.append(m)

			if (len(good) >= 30):
				print(self.object_coordinates(kp2,good))

				return self.object_coordinates(kp2,good)

	def learnbackground(self,imagem):
		if not self.tembackground:
			self.background = imagem
			self.tembackground = True
			print('Learnt Background')

	def learnobject(self,imagem):
		if self.tembackground and not self.temobjeto:
			self.objeto = self.crop_object(self.background,imagem)
			self.temobjeto = True
			self.setupfinder()
			print('Learnt object')

	def reset(self):
		self.temobjeto = False
		self.tembackground = False

	def setupfinder(self):
		if self.temobjeto and not self.setup:
			self.sift = cv2.xfeatures2d.SIFT_create()

			self.kp1, self.des1 = self.sift.detectAndCompute(self.objeto,None)

			FLANN_INDEX_KDTREE = 0
			index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
			search_params = dict(checks = 50)

			self.flann = cv2.FlannBasedMatcher(index_params, search_params)

			self.setup = True

########################################################################################################################
########################################################################################################################
# DESCOMENTA DAQUI PRA BAIXO PRA TESTAR NO pc
########################################################################################################################
########################################################################################################################


# cap = cv2.VideoCapture(0)
#
# # learned = False
# # hasBG = False
#
# counter = 0
#
# # def find_object(BG,objonBG):
# # 	img_get = cv2.subtract(objonBG,BG)
# # 	(idx,idy) = np.where((img_get[:,:,0] > 50) & (img_get[:,:,1] > 50) & (img_get[:,:,2] > 50))
# # 	(topx,topy) = (np.min(idx),np.min(idy))
# # 	(botx,boty) = (np.max(idx),np.max(idy))
# # 	return objonBG[topx:botx + 1, topy:boty + 1]
#
# obj = objeto()
#
# while True:
#
# 	ret, frame_pure = cap.read()
#
# 	if (frame_pure.any() != None):
# 		if counter != 0:
# 			if counter%100 == 0.0:
# 				obj.learnbackground(frame_pure)
# 			if counter%150 == 0.0:
# 				obj.learnobject(frame_pure)
# 	if obj.temobjeto:
# 		cv2.imshow('objeto', obj.objeto)
# 	obj.findobject(frame_pure)
# 	counter += 1
#
# 	cv2.imshow('frame puro',frame_pure)
#
# 	if cv2.waitKey(1) & 0xFF == ord('q'):
# 		break
# 	elif cv2.waitKey(1) & 0xFF == ord('r'):
# 		obj.reset()
#
# cap.release()
# cv2.destroyAllWindows()
