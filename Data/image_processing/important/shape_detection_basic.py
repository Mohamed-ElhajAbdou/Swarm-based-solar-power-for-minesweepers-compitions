import cv2
import numpy as np
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret,frame = cap.read()
    ####################################____________________Converting the image to GrayScale__________#########################################
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    resized = cv2.resize(gray,( 400,400), interpolation=cv2.INTER_AREA)

    edged = cv2.Canny(gray, 90, 175)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.dilate(edged, None, iterations=1)
    hullImage = np.zeros(edged.shape[:2], dtype="uint8")

    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    pixelsPerMetric = None
    for c in cnts:
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) <10000  :
          continue
        hull = cv2.convexHull(c)
        hullArea = cv2.contourArea(hull)
        area = cv2.contourArea(c)
        (x, y, w, h) = cv2.boundingRect(c)
        aspectRatio = w / float(h)
        extent = area / float(w * h)
        if  aspectRatio >= 1 and aspectRatio <= 3:
         if w < 400 or  h < 400:

            print aspectRatio

            #print w, h
            cv2.drawContours(hullImage, [hull], -1, 255, -1)
            cv2.drawContours(hullImage, [hull], -1, 255, -1)
            cv2.drawContours(frame, [c], -1, (240, 0, 159), 3)
            w = int(w), int(h)
            # draw the shape name on the image
            cv2.putText(frame, (str(w)), (x - 20, y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv2.imshow("imgOriginal(1)", frame)  # show windows
    cv2.imshow("Convex Hull", hullImage)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break


cap.release()
