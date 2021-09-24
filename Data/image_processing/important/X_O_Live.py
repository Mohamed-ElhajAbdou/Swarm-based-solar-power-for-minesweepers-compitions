# import the necessary packages
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
# load the tic-tac-toe image and convert it to grayscale
#image = cv2.imread("imagestictactoe.png")
    image = cv2.imread("imagestictactoe.png")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#gray_image = cv2.convertScaleAbs(gray)

# find all contours on the tic-tac-toe board
#(cnts, _) = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cv2.findContours(gray.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1]
    print cnts

# loop over the contours
    for (i,c) in enumerate(cnts):

        # compute the center of the contour
        M = cv2.moments(c)

        if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
        else:
                cX, cY = 0, 0
        area = cv2.contourArea(c)
        (x, y, w, h) = cv2.boundingRect(c)
        hull = cv2.convexHull(c)
        hullArea = cv2.contourArea(hull)
        if hullArea>0:
            solidity = (area) / float(hullArea)
        # initialize the character text
            char = "?"

        # if the solidity is high, then we are examining an `O`
            if solidity > 0.9:
                char = "O"

        # otherwise, if the solidity it still reasonable high, we
        # are examining an `X`
            elif solidity > 0.5:
                char = "X"

        # if the character is not unknown, draw it
            if char != "?":
                cv2.drawContours(image, [c], -1, (0, 255, 0), 3)
                cv2.putText(image, char, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.25,
                            (0, 255, 0), 4)

        # show the contour properties
            #print "%s (Contour #%d) -- solidity=%.2f" % (char, i + 1, solidity)

    cv2.imshow('frame', frame)
    # cv2.imshow('mask', mask)
    # cv2.imshow('result', res)






    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()