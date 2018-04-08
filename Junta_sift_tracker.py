#-*- coding: utf-8 -*-
import numpy as np
from matplotlib import pyplot as plt
import time
import sys
import cv2

import rospy
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cv_image = None


##Todo: ros compliant
#cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#time.sleep(0.5)

#------------Configuracao do SIFT ----------
MIN_MATCH_COUNT = 50

img1 = cv2.imread('alac2.jpg',0)# Imagem a procurar

sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1,None)

ok = True

#------------ Configuracao do tracker -------------
#Seleciona o tipo de tracking algorithm
def create_tracker():
	tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
	tracker_type = tracker_types[4]
	if tracker_type == 'BOOSTING':
		tracker = cv2.TrackerBoosting_create()
	if tracker_type == 'MIL':
		tracker = cv2.TrackerMIL_create()
	if tracker_type == 'KCF':
		tracker = cv2.TrackerKCF_create()
	if tracker_type == 'TLD':
		tracker = cv2.TrackerTLD_create()
	if tracker_type == 'MEDIANFLOW':
		tracker = cv2.TrackerMedianFlow_create()
	if tracker_type == 'GOTURN':
		tracker = cv2.TrackerGOTURN_create()

	return tracker,tracker_type

tracker,tracker_type = create_tracker()
#ok, frame = cap.read()
#Primeiras coordenadas da Bounding box (manualmente)
bbox = (0, 100, 200, 200) #Caixa inicial((topo esquerdo),largura,altura)

# select a bounding box via GUI
#bbox = cv2.selectROI(frame, False)

# Initialize tracker with first frame and bounding box
#ok = tracker.init(frame, bbox)



#------Inicio do Loop de atualização
contador = 0
def recebe_imagem(imagem):
	global contador


	frame = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
	vai(frame,contador)
	contador += 1
	if contador > 30:
		contador = 0

def vai(frame, contador):
	global ok,tracker,tracker_type,bbox

	if(contador == 0): #Every time the counter gets reset Try to find the café extra forte
		# Copy the image to leave the colored one to be used as output
		frame_gray = frame.copy()
		# Convert the frame to grayscale
		frame_gray = cv2.cvtColor(frame_gray, cv2.COLOR_BGR2GRAY)

		#Actual sift run
		kp2, des2 = sift.detectAndCompute(frame_gray,None)
		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)
		# Configura o algoritmo de casamento de features
		flann = cv2.FlannBasedMatcher(index_params, search_params)
		# Tenta fazer a melhor comparacao usando o algoritmo
		matches = flann.knnMatch(des1,des2,k=2)
		# store all the good matches as per Lowe's ratio test.
		good = []
		for m,n in matches:
			if m.distance < 0.7*n.distance:
				good.append(m)
		if len(good)>MIN_MATCH_COUNT:

			print('ACHEEEEEEEEEEEEEEEEEEEEEEEEEEEEEI a foooolhaaa')
			font = cv2.FONT_HERSHEY_SIMPLEX
			src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
			dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


			# Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
			M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
			#Transforma-os em pontos no espaço
			h,w = img1.shape
			pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

			# Transforma os pontos da imagem origem para onde estao na imagem destino
			dst = np.int32(cv2.perspectiveTransform(pts,M))
			# Desenha as linhas
			#cv2.polylines(frame,dst,True,(0,0,255),3, cv2.LINE_AA)
			#desenha o centro do polígono
			#top_left = dst[0][0]
			#top_right = dst[3][0]
			#bot_right = dst[2][0]
			#bot_left = dst[1][0]
			all_x = dst[:,0,0]
			all_y = dst[:,0,1]
			maxY = np.max(all_y)
			minY = np.min(all_y)

			maxX = np.max(all_x)
			minX = np.min(all_x)

			cv2.circle(frame, (minX,minY), 15, (0, 255, 0), 6)
			cv2.circle(frame,(maxX,maxY) , 15, (255, 0, 255), 6)
			if((maxX-minX)> 15 and (maxY-minY)>15):
				bbox = (minX,minY,(maxX-minX),(maxY-minY))
				tracker, tracker_type = create_tracker()
				ok = tracker.init(frame,bbox)
			else:
				print("IIhh rapah")
			print(bbox)
			cv2.imshow("Tracking", frame)
			#print(ok, "Qualqure")
			#cv2.rectangle(frame, (minX,maxY), (maxX,minY), (255,0,0), 2, 1)
			pol_y = np.int32((dst[1][0][1] - dst[0][0][1])/2 + dst[0][0][1])
			pol_x = np.int32((dst[3][0][1] - dst[0][0][0])/2 + dst[0][0][1])

	else: # Read a new frame for 30 times

		if ok:
			ok, bbox = tracker.update(frame)
			# Draw bounding box
			if ok:
				# Tracking success
				p1 = (int(bbox[0]), int(bbox[1]))
				p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
				cv2.rectangle(frame, p1, p2, (0,255,0), 3, 3)
				#Bota um circulo no centro da box
				coordx = int(p2[0]+((p1[0]-p2[0])/2))
				coordy = int(p2[1]+(p1[1]-p2[1])/2)
				cv2.circle(frame,(coordx,coordy),2,(255,0,0),1)
			else :
				# Tracking failure
				cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
				bbox = (0,0,0,0)

		# Display result
		cv2.imshow("Tracking", frame)

		k = cv2.waitKey(1) & 0xff
		if k == 27 :
			cap.release()
			cv2.destroyAllWindows()




if __name__=="__main__":
	rospy.init_node("sla")

	# Para usar a Raspberry Pi
	topico_raspberry_camera = "/raspicam_node/image/compressed"
	# Para usar a webcam
	topico_webcam = "/cv_camera/image_raw/compressed"

	recebedor = rospy.Subscriber(topico_webcam, CompressedImage, recebe_imagem, queue_size=15, buff_size = 2**24)
	print("Usando Webcam")
	velocidade_saida = rospy.Publisher("cmd_vel", Twist, queue_size = 1)

	try:
		while not rospy.is_shutdown():
			vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			if(bbox == (0,0,0,0)):
				print("BBox invalida")
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
			else:
				centro = ((bbox[0] + bbox[2]/2),(bbox[1]+ bbox[-1]/2))

				if(centro[0] < 280):
					print("esquerda")
					vel = Twist(Vector3(0,0,0),Vector3(0,0,(280-centro[0])/100))
				elif(centro[0] > 380):
					print("Direita")
					vel = Twist(Vector3(0,0,0), Vector3(0,0,-(centro[0]-380)/100))
				else:
					if(bbox[-1] > 240):
						print("And now we rest")
						vel = Twist(Vector3(0,0,0),Vector3(0,0,0))
					else:
						print("Foward we gooo!")
						vel = Twist(Vector3(0.5,0,0),Vector3(0,0,-(centro[0]-320)/300))

			velocidade_saida.publish(vel)
			rospy.sleep(0.01)

	except rospy.ROSInterruptException:
		print("Ocorreu uma exceção com o rospy")


	#except rospy.ROSInterruptException:
	#	print("Ocorreu uma exceção com o rospy")
