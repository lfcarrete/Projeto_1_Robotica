import cv2
import numpy as np
from matplotlib import pyplot as plt


# VideoCapture = cv2.VideoCapture("vid1.mp4")
# while VideoCapture.isOpened():
#     ret, frame = VideoCapture.read()

def detect_lines(frame):
    global xfuga
    teste_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Apply edge detection method on the image 
    edges = cv2.Canny(teste_gray,50,150,apertureSize = 3) 

    # Detect points that form a line

    lines = cv2.HoughLines(edges, 1, np.pi/180, 200)
    line1 = False
    line2 = False
    m1 = 0
    h1 = 0
    m2 = 0
    h2 = 0

    xfuga = None

    # Draw lines on the image
    if lines is not None:
        for linha in lines:
            for r,theta in linha: 
                a = np.cos(theta) 
                b = np.sin(theta)  
                x0 = a*r 
                y0 = b*r 
                x1 = int(x0 + 1000*(-b))  
                y1 = int(y0 + 1000*(a))  
                x2 = int(x0 - 1000*(-b))  
                y2 = int(y0 - 1000*(a)) 
                if (x2-x1)!=0:
                    m=(y2-y1)/(x2-x1)
                if m < -0.25 and m > -3:
                    if line1==False:
                        line1=True
                        m1 = m
                        h1 = y1 - m * x1
                        cv2.line(frame,(x1,y1), (x2,y2), (255,0,0),2)
                elif m > 0.25 and m < 3:
                    if line2 == False:
                        line2 = True
                        m2 = m
                        h2 = y1 - m * x1
                        cv2.line(frame,(x1,y1), (x2,y2), (255,0,0),2)
        if (m1-m2)!=0:                    
            xfuga = -1 * int((h2-h1)/(m1-m2))
        # yfuga = int(m1 * xfuga + h1) 
        # cv2.circle(frame,(xfuga,yfuga), 10, (0,0,0), -1)

        # cv2.imshow('frame', frame)
        # cv2.waitKey(1)

        return xfuga