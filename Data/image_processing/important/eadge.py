import cv2
import numpy as np
from matplotlib import pyplot as plt
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    Conv_hsv_Gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(Conv_hsv_Gray, (3, 3), 0)
    #ret,thresh = cv2.threshold(blur, 127, 255, 0)




    ret,thresh1 = cv2.threshold(blur, 50 , 255, 0)
    ret,thresh = cv2.threshold(Conv_hsv_Gray, 175, 255, 0)
    ret,thresh2 = cv2.threshold(blur, 175 , 255, 0)
    ret,thresh3 = cv2.threshold(blur, 100 , 255, 0)




    edges = cv2.Canny(thresh2,100,200)
    edges1 = cv2.Canny(thresh1,100,200)
    edges2 = cv2.Canny(thresh3,100,200)





    cv2.imshow('thresh(50-255)',edges1)
    cv2.imshow('thresh(175-255)r',edges)
    cv2.imshow('thresh(100-255)r',edges2)


    cv2.imshow('frame',frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break
cap.release()

