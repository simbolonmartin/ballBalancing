import time
import cv2


cam = cv2.VideoCapture(0)

while (cam.isOpened()):  # wait until the camera is
          
            startTime = time.time()
            ret, img = cam.read()
            # imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            fps = 1/(time.time() -  startTime)
            print(fps)
            