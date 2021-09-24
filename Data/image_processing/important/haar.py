import numpy as np
import cv2

face_cascade = cv2.CascadeClassifier('E:\Open_CV_NEW\OPEN_CV\Open_CV\opencv\build\etc\haarcascades\haarcascade_frontalface_default.xml')

img = cv2.imread('len_top.jpg')
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

faces = face_cascade.detectMultiScale(gray)

for (x,y,w,h) in faces:
    cv2.Rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = img[y:y+h, x:x+w]
    eyes = eye_cascade.detectMultiScale(roi_gray)
    for(ex,ey,ew,eh) in eyes:
        cv2.Rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

cv2.imshow('img',img)
cv2.waitKey(0)
cv2.destroyAllWindows()