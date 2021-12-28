"""
Simply display the contents of the webcam with optional mirroring using OpenCV 
via the new Pythonic cv2 interface.  Press <esc> to quit.
"""

import cv2


def show_webcam():
    cam = cv2.VideoCapture(0)
    while True:
        ret_val, img = cam.read()
        img = img[:, 50:560]
        imgHSV = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        cv2.line(img, (0, 230), (640, 230), (0, 255, 0), 2)   #horizontal line
        cv2.line(img, (253, 0), (253, 480), (0, 255, 0), 2)   #vertical line
        centerX= 253
        centerY= 230 

        # centerY = #change y to 0.1, so the target become 0.1 from 0
        sideLength = 50
        cv2.rectangle(img, (centerX-sideLength, centerY-sideLength), (centerX+sideLength, centerY+sideLength), (255, 0, 0), 2)
        # cv2.imshow('my webcam', imgHSV)
        cv2.imshow('my webcam', img)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit
    cv2.destroyAllWindows()


def main():
    show_webcam()


if __name__ == '__main__':
    main()