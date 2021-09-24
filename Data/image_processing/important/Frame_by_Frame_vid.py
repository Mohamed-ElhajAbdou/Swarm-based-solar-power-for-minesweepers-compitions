import cv2
import numpy as np
import os
from matplotlib import pyplot as plt


cap = cv2.VideoCapture(0)
while cap.isOpened():
    ret,frame = cap.read()
    warp = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ORANGE_MIN = np.array([40, 40, 60], np.uint8)
    ORANGE_MAX = np.array([50, 70, 80], np.uint8)

    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Conv_hsv_Gray = cv2.cvtColor(hsv_img, cv2.COLOR_BGR2GRAY)

    framethreshed = cv2.inRange(hsv_img, ORANGE_MIN, ORANGE_MAX)
    ret, mask = cv2.threshold(Conv_hsv_Gray,100, 150, cv2.THRESH_BINARY)

    res = cv2.bitwise_and(Conv_hsv_Gray, Conv_hsv_Gray, mask=mask)


############################################################################
    pts1 = np.float32([[30, 300], [390, 400], [0, 0], [300, 30]])

    # pts1 = np.float32([[65, 50], [300, 50], [50,300], [390, 300]])
    # pts1 = np.float32([ [389, 390], [368, 52], [28, 387],[56, 65]])
    pts2 = np.float32([[0, 300], [300, 300], [0, 0], [300, 0]])

    M = cv2.getPerspectiveTransform(pts1, pts2)

    dst = cv2.warpPerspective(res, M, (800, 800))
######################################################################

    imf = np.float32(dst) / 255.0  # float conversion/scale

    Desc = cv2.dct(imf)

    # kernel = np.ones((100, 100), np.float32) / 1000
    # Fitr = cv2.filter2D(Desc, -1, kernel)
    Fitr = cv2.blur(Desc, (1, 1))

    Filter2 = cv2.GaussianBlur(Desc, (1, 1), 0)
    iddt1 = cv2.idct(Fitr)
    iddt2 = cv2.idct(Filter2)

    cv2.imwrite('output2.jpg', framethreshed)
    ###############################################
    corners = cv2.goodFeaturesToTrack(iddt1, 70, 0.1, 20)

    corners = np.int0(corners)

    for i in corners:
        x, y = i.ravel()
        cv2.circle(Conv_hsv_Gray, (x, y),7, 200, -1)
#######################################

    cv2.imshow("imgOriginal(1)", frame)  # show windows
    cv2.imshow("Masking res(3)", res)  # show windows
    #cv2.imshow("Conv_hsv_Gray(2)", Conv_hsv_Gray)  # show windows


    #cv2.imshow("hsv_img(4)", hsv_img)  # show windows

    #cv2.imshow("imgCanny", imgCanny)
    #cv2.imshow("Gray(5)", warp)
    #cv2.imshow("resizing", res)
    #cv2.imshow("Warping(6)", dst)
    #cv2.imshow("Filter1(8.1)", Fitr)
    #cv2.imshow("Filter2(8.2)", Filter2)

    #cv2.imshow("DCT(7.1)", Desc)
    #cv2.imshow("IDCT1(7.2)", iddt1)
    cv2.imshow("IDCT2(9)", iddt2)
   # cv2.imshow("frame_threshed(9)", framethreshed)

    if cv2.waitKey(20) & 0xFF == ord('q'):
        break


cap.release()
