import cv2
import numpy as np
from functions import *
def nothing(x):
    pass
cap = cv2.VideoCapture(0)
# face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_alt.xml')

#face_cascade = cv2.CascadeClassifier('E:\\Open_CV_NEW\\OPEN_CV\\Open_CV\\opencv\\build\\etc\\haarcascades\\haarcascade_frontalface_default.xml')


face_cascade = cv2.CascadeClassifier('C:\Users\El Holandy\Desktop\image_Procesing\opencv-master\data\haarcascades\haarcascade_frontalface_default.xml')

#eye_cascade = cv2.CascadeClassifier('C:\opencv\build\\etc\haarcascades\haarcascade_eye.xml')

cv2.namedWindow('Video')
cv2.moveWindow('Video',5,5)
cv2.namedWindow('HSV_Thresh')
cv2.moveWindow('HSV_Thresh',655,5)
cv2.createTrackbar('tval', 'Video', 29, 255, nothing)
cv2.createTrackbar('htoler', 'HSV_Thresh', 17, 100, nothing)
cv2.createTrackbar('stoler', 'HSV_Thresh', 25, 100, nothing)
cv2.createTrackbar('vtoler', 'HSV_Thresh', 84, 100, nothing)

kernel = np.ones((5, 5), np.uint8)# 5X5 erosion kernel
bavg=0
ravg=0
gavg=0
while True:
    tval1=cv2.getTrackbarPos('tval', 'Video')#thresh value to remove non skin components from face
    htoler_val=cv2.getTrackbarPos('htoler', 'HSV_Thresh')
    stoler_val=cv2.getTrackbarPos('stoler', 'HSV_Thresh')
    vtoler_val=cv2.getTrackbarPos('vtoler', 'HSV_Thresh')
    ret,img=cap.read()#Read from source
    img[0:100,0:100] = [255,255,255]
    thresh_hsv_toler=img
    faces = face_cascade.detectMultiScale(img, 1.3, 5)
    for (x,y,w,h) in faces:
        bavg=0
        ravg=0
        gavg=0
        numpix=0
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        roi_face = img[y:y+h, x:x+w]
        #avg_col=img[100,100]
        rect_face=img[y:y+h-h/8,x+w/7:x+w-w/5]#extract only skin features from remaining bg
        mask=cv2.inRange(rect_face,(tval1,tval1,tval1),(255,255,255))
        mask=cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        tone=cv2.subtract(mask,rect_face)
        tone=cv2.subtract(mask,tone)
        (rows,cols,col)=tone.shape # 480 rows and 640 cols; 3 values for RGB img
        for i in range(rows): #note the presence of colon
            for j in range(cols):
                if (tone[i,j,0]!=0 and tone[i,j,0]!=0 and tone[i,j,0]!=0):
                    bavg=bavg+tone[i,j,0]
                    gavg=gavg+tone[i,j,1]
                    ravg=ravg+tone[i,j,2]
                    numpix=numpix+1
                    bavg=bavg/numpix
                    gavg=gavg/numpix
                    ravg=ravg/numpix
        '''print "bavg="+str(bavg)
        print "gavg="+str(gavg)
        print "ravg="+str(ravg)
        print "numpix="+str(numpix)'''
        cv2.circle(img, (50,50), 20, (bavg,gavg,ravg), 50)#get obtained average colour on screen


        cv2.imshow('skin_mask', tone)
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        thresh_hsv_toler=cv2.inRange(hsv,(hsv[50,50,0]-htoler_val,hsv[50,50,1]-stoler_val,hsv[50,50,2]-vtoler_val),(hsv[50,50,0]+htoler_val,hsv[50,50,1]+stoler_val,hsv[50,50,2]+vtoler_val))

        thresh_hsv_toler=cv2.dilate(thresh_hsv_toler, kernel, iterations=1)
        thresh_hsv_toler=cv2.cvtColor(thresh_hsv_toler,cv2.COLOR_GRAY2BGR)#superimposing binary mask on image
        hsv_filter=cv2.subtract(thresh_hsv_toler,img)
        hsv_filter=cv2.subtract(thresh_hsv_toler,hsv_filter)



        cv2.imshow('HSV_Thresh', hsv_filter)


    if(cv2.waitKey(10) & 0xFF == ord('b')):
            break
    cv2.imshow('Video', img)