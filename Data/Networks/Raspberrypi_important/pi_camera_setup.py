# import the necessary packages
#!/usr/bin/python

import RPi.GPIO as GPIO
import time
import spidev
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import numpy
import imutils
from lib_nrf24 import NRF24
import spi
import struct
from math import sqrt
import os
import math

GPIO.setmode(GPIO.BCM)

pipes = [[ 0xE8, 0xE8, 0xF0, 0xF0, 0xE1],[0xF0,0xF0,0xF0,0xF0,0xE1]]

radio=NRF24(GPIO, spidev.SpiDev())
radio.begin(0,17)

radio.setPayloadSize(32)
radio.setChannel(0x76)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)

radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1,pipes[1])
radio.printDetails()

def minimum_value(pair, dictionary={}):  # intentional dangerous default value
    if pair not in dictionary:
        #dictionary[pair] = pair[0] ** 2 + pair[1] ** 2
        dictionary[pair]=sqrt(pow(pair[0], 2) + pow(pair[1], 2))
    else :
        print ("error")
    return dictionary[pair]
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 80
rawCapture = PiRGBArray(camera, size=(640, 480))
 
# allow the camera to warmup
time.sleep(1)
 
# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

        
        ORANGE_MIN = np.array([     40,     40,     60], np.uint8)
        ORANGE_MAX = np.array([50, 70, 80], np.uint8)
        lower_blue = np.array([110, 50, 50])
        upper_blue = np.array([130, 255, 255])
        Lower_black= np.array([0,0,0])
        upper_black= np.array([140, 140, 140])


        
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)
        mask_inv = cv2.bitwise_not(Conv_hsv_Gray)
        mask = cv2.inRange(hsv_img, lower_blue, upper_blue)
    # imf = np.float32(Conv_hsv_Gray) / 255.0  # float conversion/scale
        imf = np.float32(mask) / 255.0  # float conversion/scale

        Desc = cv2.dct(imf)
        sub_to = Desc[0, 0]
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
        dict = {}
        obj_list = []
        dict = []
        w, h = 2,2
        Mines_locations = [[0 for x in range(h)] for y in range(w)]
        bla=[]
        bla1=[]
        bla2=[]
        L=[]
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
            p=int(x)/(6.2*5*2)
            w=int(y)/(4.8*5*2)
            XX=int(x)/(6.2*2*2*5)
            YY=int(y)/(4.8*2*2*5)
            s=int(XX),int(YY)
 
            if radius>50 and radius<300 :
            
             ((x, y), radius) = cv2.minEnclosingCircle(c)

             time.sleep(0.35)
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
             #dict=(int(XX), int(YY))
             dict.append((int(XX),int(YY)))
             #Mines_locations=dict
             yarab = {k: v for k, v in zip(List11, obj_list)}
             #yarab = {k: v for k, v in zip(List11, dict)}
             obj_list.append((int(x), int(y)))
             yarab={k: v for k, v in zip(List11, obj_list)}
             #############################
             #########################                    Applying the Minmum Path Sorting               #####################################################################
             bla = sorted(dict, key=minimum_value)
             bla1 = sorted(dict, key=minimum_value)
             x3 = [[0 for col in range(len(bla))] for row in range(2)]
             print (x3)
             bla1 = zip(*bla1)
             j=0
             i=0
             for i in range(0, len(bla)):
                for j in range(2):
                    if bla1[j][i] == 1:
                        x3[j][i] = 'A'
                        # print 11111, i, j
                    if bla1[j][i] == 2:
                        x3[j][i] = 'B'
                        # print 22222, i, j
                    if bla1[j][i] == 3:
                        x3[j][i] = 'C'
                        # print 3333, i, j
                    if bla1[j][i] == 4:
                        x3[j][i] = 'D'
                        # print 4444, i, j
                    if bla1[j][i] == 5:
                        x3[j][i] = 'E'
                        # print 5555, i, j
             print ("no zip", bla)
             print ("ziped",bla1)
            #print obj_list
             x3 = zip(*x3)
             print  ("unziped",x3)
             message=list(x3)
             print (message)
            #x3 = zip(*x3)
            #print "ziped again", x3
            #############################
             #print (int(x),int(y))

             
 
             #print s
             





             cv2.circle(image, (int(x), int(y)), int(radius),
                      (0, 0, 255), 2)
             cv2.circle(image, (cX, cY), 2, (255, 255, 255), -1)

             cv2.putText(image,(str(s)) , (cX - 20, cY - 20),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            


        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(10) & 0xFF
 
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        #time.sleep(1)
        #spi.closeSPI()
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
                break



        
