import cv2
import numpy as np
import numpy
import imutils
from math import sqrt
import os
import math

M = [(20, 10), (3, 4), (5, 6), (1.2, 7), (6.5, 4)]


def minimum_value(pair, dictionary={}):  # intentional dangerous default value
    if pair not in dictionary:
        # dictionary[pair] = pair[0] ** 2 + pair[1] ** 2
        dictionary[pair] = sqrt(pow(pair[0], 2) + pow(pair[1], 2))


    else:
        print "error"

    return dictionary[pair]


cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    # frame = cv2.resize(frame1, (600, 600))
    #################################____________________Converting the Image to HSV and MASKING the image ______________________#####################################
    ORANGE_MIN = np.array([40, 40, 60], np.uint8)
    ORANGE_MAX = np.array([50, 70, 80], np.uint8)
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv_img', hsv_img)
    Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
    H,S,V = cv2.split(hsv_img)
    Conv_frame_Gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('Conv_frame_Gray', Conv_frame_Gray)
    # cv2.imshow('Conv_hsv_Gray', Conv_hsv_Gray)
    mask_inv = cv2.bitwise_not(Conv_hsv_Gray)
    # cv2.imshow('mask_inv.bitwise_not', mask_inv)
    # define range of blue color in HSV
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([130, 220, 200])
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([170, 220, 200])
    # Lower_black= np.array([0, 0, 0])
    Lower_black = np.array([0, 0, 0])

    # upper_black= np.array([180, 255, 40])
    # upper_black= np.array([140, 140, 150])
    # upper_black= np.array([140, 140, 140])
    upper_black = np.array([140, 140, 140])

    # lower_white = np.array([0,0,0])
    # upper_white = np.array([0,0,200])
    sensitivity = 20
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])

    mask = cv2.inRange(hsv_img, lower_blue, upper_blue)

    # mask = cv2.inRange(hsv_img, Lower_black, upper_black)
    # sub_to2=sub_to**2
    # Desc2=Desc**2
    # result=Desc2-sub_to2
    # xeco=np.sqrt(result)
    ###########################################################Filtring the image ######################################################################################
    imf = np.float32(S) / 255.0  # float conversion/scale
    imfo = np.float32(mask) / 255.0  # float conversion/scale
    Desc = cv2.dct(imf)
    Desc[0,0]=0
    Fitr = cv2.blur(Desc, (1, 1))
    iddt1 = cv2.idct(Fitr)
    res = cv2.bitwise_and(iddt1, iddt1, mask=mask)
    kernel = np.ones((5, 5), np.uint8)
    erosion = cv2.erode(res, kernel, iterations=2)
    cv2.imshow('resuult', res)
    dilation = cv2.dilate(erosion, kernel, iterations=2)
    opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel)

    # cv2.imshow('dilation', dilation)
    # cv2.imshow('opening', opening)


    # find contours in the thresholded image
    # cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cvuint8_res = cv2.convertScaleAbs(res)

    #print mask.dtype
    # find contours in the thresholded image
    # cnts = cv2.findContours(cvuint8_res.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cv2.findContours(cvuint8_res.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    #cnts2 = cnts2[0] if imutils.is_cv2() else cnts2[1]

    cX = [len(cnts)]
    #cX2 = [len(cnts2)]

    cY = [len(cnts)]
    #cY2 = [len(cnts2)]

    obj_list = []
    dict = []
    w, h = 2, 2
    Mines_locations = [[0 for x in range(h)] for y in range(w)]
    bla = []
    L = []

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
        XX = int(x) / 6.2
        YY = int(y) / 4.8
        w = int(XX), int(YY)
        L = [(int(XX), int(YY))]
        bla1 = []

        # draw the contour and center of the shape on the image
        # if cX >10 and cY>10:

        if radius > 50 and radius < 700:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            List2 = ([cX])
            List3 = ['X-Coord']
            fofo = {k: v for k, v in zip(List3, List2)}
            List4 = ([cY])
            List5 = ['Y-Coord']
            gogo = {k: v for k, v in zip(List4, List5)}
            List6 = ([fofo])
            List7 = ([gogo])
            List8 = [List6, List7]

            aTargetDictionary = {}
            List10 = [(radius)]
            # List11 = ['obj1','obj2','obj3','obj4','obj5','obj6','obj7','obj8','obj9','obj10','obj11','obj12','obj13','obj14','obj15','obj16','obj17','obj18','obj19','obj20' \
            #          ' obj21','obj22','obj23','obj24','obj25','obj26','obj27','obj28','obj29','obj30','obj31','obj32','obj33',
            #           'obj34','obj35','obj36','obj37','obj38','obj39','obj40']
            List11 = ['obj1', 'obj2', 'obj3', 'obj4', 'obj5']

            obj_list.append((int(x), int(y)))
            # dict=(int(XX), int(YY))
            dict.append((int(XX), int(YY)))
            Mines_locations = dict
            yarab = {k: v for k, v in zip(List11, obj_list)}
            #########################                    Applying the Minmum Path Sorting               #####################################################################
            bla = sorted(dict, key=minimum_value)
            print bla
            #bla1 = zip(*bla1)
            #print  bla[0][1],bla1[1][0]


            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 0, 255), 2)
            cv2.circle(frame, (cX, cY), 2, (255, 255, 255), -1)

            cv2.putText(frame, (str(w)), (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            # cv2.putText(frame, "center x,y".format(cX, cY), (cX - 20, cY - 20),
            #                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    #cv2.imshow('iddt1', iddt1)
    cv2.imshow('res frame AfterALL ', frame)


    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
