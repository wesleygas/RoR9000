#!/usr/bin/env python
# -*- coding:utf-8 -*-
import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

# If you want to open a video, just change this path
#cap = cv2.VideoCapture('hall_box_battery.mp4')

# Parameters to use when opening the webcam.
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


MIN_MATCH_COUNT = 60

img1 = cv2.imread('alac2.jpg',0)# Imagem a procurar

sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1,None)

while(True):
    ret, img2 = cap.read()
    # Save an colored image to use as output
    img2_color = img2.copy()
    # Convert the frame to grayscale
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

    #Depois de detectar os círculos, passamos as imagens originais para serem comparadas pelo sift
    #kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
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

        #print('ACHEEEEEEEEEEEEEEEEEEEEEEEEEEEEEI a foooolhaaa')
        font = cv2.FONT_HERSHEY_SIMPLEX
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)


        # Tenta achar uma trasformacao composta de rotacao, translacao e escala que situe uma imagem na outra
        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        matchesMask = mask.ravel().tolist()

        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        # Transforma os pontos da imagem origem para onde estao na imagem destino
        dst = cv2.perspectiveTransform(pts,M)
        # Desenha as linhas
        cv2.polylines(img2_color,[np.int32(dst)],True,(0,0,255),3, cv2.LINE_AA)

        #desenha o centro do polígono
        #pol_y = np.int32((dst[1][0][1] - dst[0][0][1])/2 + dst[0][0][1])
        #pol_x = np.int32((dst[3][0][1] - dst[0][0][0])/2 + dst[0][0][1])


    # Display the resulting frame
    cv2.imshow('Detector de circulos',img2_color)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
