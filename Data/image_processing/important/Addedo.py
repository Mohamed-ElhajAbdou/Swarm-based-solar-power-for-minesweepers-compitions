import cv2
import numpy as np
import numpy
import imutils
from math import sqrt
import os
import math
green = np.uint8([[[0,255,0 ]]])
hsv_green = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)


cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    # frame = cv2.resize(frame1, (600, 600))
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)

    H,S,V = cv2.split(hsv_img)
    cv2.imshow('res frame AfterALL ', frame)
    cv2.imshow(' H ', H)
    cv2.imshow(' S ', S)
    cv2.imshow(' V ', V)

    cv2.imshow('sv_Gray ', Conv_hsv_Gray)



    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
