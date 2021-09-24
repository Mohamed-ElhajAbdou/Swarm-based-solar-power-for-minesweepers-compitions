import cv2
import numpy as np
import os
import numpy
from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
from multiprocessing.dummy import Pool
import itertools




cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret,frame = cap.read()
    cimg=numpy.zeros((700, 700))
    triangle = numpy.array([[1, 3], [4, 8], [1, 9]], numpy.int64)
    #################################____________________Converting the Image to HSV and MASKING the image ______________________#####################################
    ORANGE_MIN = np.array([40, 40, 60], np.uint8)
    ORANGE_MAX = np.array([50, 70, 80], np.uint8)



    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
    mask_inv = cv2.bitwise_not(Conv_hsv_Gray)


    # define range of blue color in HSV
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])

    #Lower_black= np.array([0, 0, 0])
    Lower_black= np.array([0,0,0])

    # upper_black= np.array([180, 255, 40])
    # upper_black= np.array([140, 140, 150])
    #upper_black= np.array([140, 140, 140])
    upper_black= np.array([140, 140, 140])



    # lower_white = np.array([0,0,0])
    # upper_white = np.array([0,0,200])
    sensitivity = 20
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])

    # mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
    mask = cv2.inRange(hsv_img, Lower_black, upper_black)




    ###########################################################Filtring the image ######################################################################################
    # imf = np.float32(Conv_hsv_Gray) / 255.0  # float conversion/scale
    imf = np.float32(mask) / 255.0  # float conversion/scale

    Desc = cv2.dct(imf)
    sub_to = Desc[0, 0]
    # print sub_to
    result_frame = Desc - sub_to

    Fitr = cv2.blur(result_frame, (1, 1))

    iddt1 = cv2.idct(Fitr)
    ####################################################################################################################################################################

    res = cv2.bitwise_and(iddt1, iddt1, mask=mask)

    # find contours in the thresholded image
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    cX=[len(cnts)]
    cY=[len(cnts)]
    obj_list=[]
    dict = []

    # loop over the contours
    for c in cnts:


        # compute the center of the contour
        M = cv2.moments(c)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0


        ((x, y), radius) = cv2.minEnclosingCircle(c)


        # draw the contour and center of the shape on the image
        # if cX >10 and cY>10:

        if radius>40 and radius<300 :
             ((x, y), radius) = cv2.minEnclosingCircle(c)
             List2 =( [cX])
             List3=['X-Coord']
             fofo={k: v for k, v in zip(List3, List2)}
             List4 = ([cY])
             List5 = ['Y-Coord']
             gogo = {k: v for k, v in zip(List4, List5)}
             List6=([fofo])
             List7=([gogo])
             List8=[List6,List7]

             aTargetDictionary = {}
             List10=[(radius)]
             List11 = ['obj1','obj2','obj3','obj4','obj5','obj6','obj7','obj8','obj9','obj10','obj11','obj12','obj13','obj14','obj15','obj16','obj17','obj18','obj19','obj20' \
                      ' obj21','obj22','obj23','obj24','obj25','obj26','obj27','obj28','obj29','obj30','obj31','obj32','obj33',
                       'obj34','obj35','obj36','obj37','obj38','obj39','obj40']
             obj_list.append((int(x), int(y)))
             yarab={k: v for k, v in zip(List11, obj_list)}
             #print (int(x),int(y))
             XX=int(x)/6.2
             YY=int(y)/4.8
             #s=int(x),int(y)
             int(x)
             int(YY)
             #print type (x)
             s = int(XX), int(YY)
             dict.append((int(XX), int(YY)))
             print dict
             cv2.circle(frame, (int(x), int(y)), int(radius),
                      (0, 0, 255), 2)
             cv2.circle(frame, (cX, cY), 2, (255, 255, 255), -1)
             cv2.putText(frame,(str(s)) , (cX - 20, cY - 20),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # cv2.putText(frame, "center x,y".format(cX, cY), (cX - 20, cY - 20),
            #                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.imshow('frame', frame)
    # cv2.imshow('mask', mask)
    cv2.imshow('result', res)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


cap.release()
