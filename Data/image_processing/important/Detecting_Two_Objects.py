import cv2
import numpy as np
import numpy
import imutils
from math import sqrt
import os
from time import gmtime , strftime
def minimum_value(pair, dictionary={}):  # intentional dangerous default value
    if pair not in dictionary:
        #dictionary[pair] = pair[0] ** 2 + pair[1] ** 2
        dictionary[pair]=sqrt(pow(pair[0], 2) + pow(pair[1], 2))
        # if np.array_equal(mask, blue_mask):
        #     print ('Robot', (int(XX), int(YY)))
    else :
        pass
        #print "Waiting..."
    return dictionary[pair]
#
# def dist(tuple):
#     return sqrt(pow(tuple[0], 2) + pow(tuple[1], 2))
#
# def sorting_func(first, second):
#     import pdb
#     pdb.set_trace()
#     if dist(first) < dist(second):
#         return 1
#     elif dist(second) < dist(first):
#         return -1
#     else:
#         return 0
counter =0
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret, frame = cap.read()
    #################################____________________Converting the Image to HSV and MASKING the image ______________________#####################################
    ORANGE_MIN = np.array([40, 40, 60], np.uint8)
    ORANGE_MAX = np.array([50, 70, 80], np.uint8)
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
    mask_inv = cv2.bitwise_not(Conv_hsv_Gray)
    # define range of blue color in HSV
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])


    lower_green = np.array([50, 50, 140])
    upper_green = np.array([100, 255, 255])


    Lower_black = np.array([0, 0, 0])
    # upper_black= np.array([180, 255, 40])
    # upper_black= np.array([140, 140, 150])
    # upper_black= np.array([140, 140, 140])
    upper_black = np.array([140, 140, 140])
    Lower_RED=np.array([10,255,255])
    Upper_RED=np.array([179,255,255])
    # lower_white = np.array([0,0,0])
    # upper_white = np.array([0,0,200])
    sensitivity = 20
    lower_white = np.array([0, 0, 255 - sensitivity])
    upper_white = np.array([255, sensitivity, 255])
    black_mask = cv2.inRange(hsv_img, Lower_black, upper_black)
    blue_mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
    green_mask = cv2.inRange(hsv_img, lower_green, upper_green) # I have the Green threshold image.
    red_mask = cv2.inRange(hsv_img, Lower_RED, Upper_RED) # I have the Green threshold image.
    #mask = blue_mask + black_mask
    mask = blue_mask
    # if ( counter<400):
    #     mask = green_mask
    # else :
    #     mask= blue_mask
    # find contours in the thresholded image
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    obj_list = []
    obj_list2 = []

    dict = []
    dict2 = []

    bla=[]
    bla2=[]

    L=[]
    L2=[]

    Robot=[]
    Robot2=[]

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
        L=[(int(XX),int(YY))]
        # draw the contour and center of the shape on the image
        # if cX >10 and cY>10:
        if radius >60 and radius < 100:
            ((x, y), radius) = cv2.minEnclosingCircle(c)


            aTargetDictionary = {}
            List10 = [(radius)]
            # List11 = ['obj1','obj2','obj3','obj4','obj5','obj6','obj7','obj8','obj9','obj10','obj11','obj12','obj13','obj14','obj15','obj16','obj17','obj18','obj19','obj20' \
            #          ' obj21','obj22','obj23','obj24','obj25','obj26','obj27','obj28','obj29','obj30','obj31','obj32','obj33',
            #           'obj34','obj35','obj36','obj37','obj38','obj39','obj40']
            List11 = ['obj1', 'obj2', 'obj3', 'obj4', 'obj5']




            obj_list.append((int(x), int(y)))
            dict.append((int(XX),int(YY)))
            Mines_locations=dict
            yarab = {k: v for k, v in zip(List11, obj_list)}
            #yarab = {k: v for k, v in zip(List11, dict)}

            #########################                    Applying the Minmum Path Sorting               #####################################################################
            bla = sorted(dict, key=minimum_value)




            cv2.circle(frame, (int(x), int(y)), int(radius),
                       (0, 0, 255), 2)
            cv2.circle(frame, (cX, cY), 2, (255, 255, 255), -1)

            cv2.putText(frame, (str(w)), (cX - 20, cY - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.imshow('frame', frame)
            # cv2.putText(frame, "center x,y".format(cX, cY), (cX - 20, cY - 20),
            #                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        dodo = frame
        cnts2 = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts2 = cnts2[0] if imutils.is_cv2() else cnts2[1]

            #cv2.imshow('dodo', dodo)
            ################################################_____Ya_rab####################################
        for c2 in cnts2:
                # compute the center of the contour
                M2 = cv2.moments(c2)
                if M2["m00"] != 0:
                    cX2 = int(M2["m10"] / M2["m00"])
                    cY2 = int(M2["m01"] / M2["m00"])
                else:
                    cX2, cY2 = 0, 0
                ((x2, y2), radius2) = cv2.minEnclosingCircle(c2)
                XX2 = int(x2) / 6.2
                YY2 = int(y2) / 4.8
                w2 = int(XX2), int(YY2)
                L2 = [(int(XX2), int(YY2))]
                # draw the contour and center of the shape on the image
                # if cX >10 and cY>10:
                if radius2 > 60 and radius2< 100:
                    ((x2, y2), radius2) = cv2.minEnclosingCircle(c2)
                    #print x2,y2

                    aTargetDictionary2 = {}
                    List102 = [(radius2)]
                    # List11 = ['obj1','obj2','obj3','obj4','obj5','obj6','obj7','obj8','obj9','obj10','obj11','obj12','obj13','obj14','obj15','obj16','obj17','obj18','obj19','obj20' \
                    #          ' obj21','obj22','obj23','obj24','obj25','obj26','obj27','obj28','obj29','obj30','obj31','obj32','obj33',
                    #           'obj34','obj35','obj36','obj37','obj38','obj39','obj40']
                    List112 = ['obj1', 'obj2', 'obj3', 'obj4', 'obj5']

                    obj_list2.append((int(x2), int(y2)))
                    #dict2.append((int(XX2), int(YY2)))
                    dict2.append((int(XX2), int(YY2)))
                    Mines_locations2 = dict2
                    yarab2 = {k: v for k, v in zip(List112, obj_list2)}
                    # yarab = {k: v for k, v in zip(List11, dict)}

                    #########################                    Applying the Minmum Path Sorting               #####################################################################
                    bla2 = sorted(dict2, key=minimum_value)
                    print "GREEEN", dict2

                    print "BLUE", bla

                    cv2.circle(dodo, (int(x2), int(y2)), int(radius2),
                               (0, 0, 255), 2)
                    cv2.circle(dodo, (cX2, cY2), 2, (255, 255, 255), -1)

                    cv2.putText(dodo, (str(w2)), (cX2 - 20, cY2 - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow('dodo', dodo)


                ###############################################################################################

#cv2.imshow('frame', frame)
    #counter=counter+1
    #print counter

    # cv2.imshow('mask', mask)
    # cv2.imshow('result', res)


    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
