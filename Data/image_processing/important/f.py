import cv2
import numpy as np
import numpy
import imutils
from math import sqrt
import os
import math

cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    cv2.imshow('frame', frame)

    #################################____________________Converting the Image to HSV and MASKING the image ______________________#####################################
    ORANGE_MIN = np.array([40, 40, 60], np.uint8)
    ORANGE_MAX = np.array([50, 70, 80], np.uint8)

    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow('hsv_img', hsv_img)

    Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
    Conv_frame_Gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # cv2.imshow('Conv_frame_Gray', Conv_frame_Gray)
    #
    # cv2.imshow('Conv_hsv_Gray', Conv_hsv_Gray)

    mask_inv = cv2.bitwise_not(Conv_hsv_Gray)
    # cv2.imshow('mask_inv.bitwise_not', mask_inv)


    # define range of blue color in HSV
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])

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

    ###########################################################Filtring the image ######################################################################################
    imf = np.float32(Conv_hsv_Gray) / 255.0  # float conversion/scale

    #imf = np.float32(mask) / 255.0  # float conversion/scale

    Desc = cv2.dct(imf)
    #cv2.imshow('Desc', Desc)

    sub_to = Desc[0, 0]
    # print sub_to
    #    result_frame =math.sqrt(Desc)^2 - sub_to
    # loko=((Desc)^2)-((sub_to)^2)
    #    loko=math.pow(Desc, 2)
    # result_frame=math.sqrt(loko)

    result_frame = Desc - sub_to

    #Fitr = cv2.blur(result_frame, (1, 1))
    Fitr = cv2.blur(result_frame, (1, 1))

    iddt1 = cv2.idct(Fitr)
    iddt2 = cv2.idct(result_frame)




    ####################################################################################################################################################################

    res = cv2.bitwise_and(iddt1, iddt1, mask=mask)
    res2 = cv2.bitwise_and(iddt2, iddt1, mask=mask)
    cvuint8 = cv2.convertScaleAbs(res2)
    print mask.dtype
    # find contours in the thresholded image
    cnts = cv2.findContours(cvuint8.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    #cnts2 = cv2.findContours(res2.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow('mask', mask)
    # cv2.imshow('Conv_hsv_GraybeforeDCT', Conv_hsv_Gray)
    # cv2.imshow('iddt1with LPF', iddt1)
    # cv2.imshow('iddt1without LPF', iddt2)
    # cv2.imshow('res iddt1with LPF LPF', res)
    # cv2.imshow('res2 iddt1without LPF', res2)
    # cv2.imshow('res2 mask LPF', mask)



    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
