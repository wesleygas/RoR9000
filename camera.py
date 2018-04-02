import cv2
import time
cap = cv2.VideoCapture(0)
time.sleep(1)

while True:
	
	ret, frame_pure = cap.read()
	cv2.imshow('Pure frame com circulos',frame_pure)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
