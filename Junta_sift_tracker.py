#-*- coding: utf-8 -*-
import numpy as np
from matplotlib import pyplot as plt
import time
import sys
import cv2

##Todo:
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
time.sleep(1)

#------------Configuracao do SIFT ----------
MIN_MATCH_COUNT = 60

img1 = cv2.imread('alac2.jpg',0)# Imagem a procurar

sift = cv2.xfeatures2d.SIFT_create()
kp1, des1 = sift.detectAndCompute(img1,None)



#------------ Configuracao do tracker -------------
#Seleciona o tipo de tracking algorithm
def create_track():
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

tracker,tracker_type =create_track()
ok, frame = cap.read()
#Primeiras coordenadas da Bounding box (manualmente)
bbox = (287, 23, 86, 320)

# select a bounding box via GUI
#bbox = cv2.selectROI(frame, False)

# Initialize tracker with first frame and bounding box
ok = tracker.init(frame, bbox)

#------Inicio do Loop de atualização

while True:
    ret, img2 = cap.read()
    # Save an colored image to use as output
    img2_color = img2.copy()
    # Convert the frame to grayscale
    img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

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

        print('ACHEEEEEEEEEEEEEEEEEEEEEEEEEEEEEI a foooolhaaa')
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
        maxY = np.int32(dst[1][0][1])
        minY = np.int32(dst[0][0][1])

        maxX = np.int32(dst[3][0][1])
        minX = np.int32(dst[0][0][0])
        cv2.circle(frame, (minX,minY), 15, (255, 0, 0), 6)
        cv2.circle(frame, (maxX,maxY), 15, (255, 0, 0), 6)
        tracker, tracker_type = create_track()
        ok = tracker.init(frame, (minX,minY,maxY,maxX+(minX+maxX)))
        cv2.imshow("Tracking", frame)
        #print(ok, "Qualqure")
        #cv2.rectangle(frame, (minX,maxY), (maxX,minY), (255,0,0), 2, 1)
        #pol_y = np.int32((dst[1][0][1] - dst[0][0][1])/2 + dst[0][0][1])
        #pol_x = np.int32((dst[3][0][1] - dst[0][0][0])/2 + dst[0][0][1])

    for i in range(1): # Read a new frame for 30 times

        if ok:

            ok, frame = cap.read()
            # Start timer
            timer = cv2.getTickCount()

            # Update tracker
            ok, bbox = tracker.update(frame)

            # Calculate Frames per second (FPS)
            fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);

            # Draw bounding box
            if ok:
                # Tracking success
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
                #Bota um circulo no centro da box
                coordx = int(p2[0]+((p1[0]-p2[0])/2))
                coordy = int(p2[1]+(p1[1]-p2[1])/2)
                cv2.circle(frame,(coordx,coordy),2,(255,0,0),1)
            else :
                # Tracking failure
                cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)

        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2);

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2);

        # Display result
        #cv2.imshow("Tracking", frame)

        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 :
            cap.release()
            cv2.destroyAllWindows()
            break

cap.release()
cv2.destroyAllWindows()
