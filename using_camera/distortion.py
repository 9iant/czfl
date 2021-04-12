import numpy as np
import cv2

#!/usr/bin/python python
def get_undistparams():
    # Undistortion parameter
    k0 = 0.800900859568945
    k2 = 0.143324176606491
    k3 = -0.149250283681958
    k4 = 0.027493140732628
    cx = 3.226759726108088e+02
    cy = 2.557579459524219e+02
    f_x = 240*1.053165 # --> f = 270으로 
    f_y = 240
    rows = 480
    cols = 640
    undistX = np.zeros((rows, cols), dtype=np.float32)
    undistY = np.zeros((rows, cols), dtype=np.float32)
    for i in range(rows):
        for j in range(cols):
            ynu = (i - (rows - cy)) / f_y
            xnu = (j - (cols - cx)) / f_x
            r = np.sqrt(pow(xnu, 2) + pow(ynu, 2))
            ynd = (k0 + k2 * pow(r, 2) + k3 * pow(r, 3) + k4 * pow(r, 4)) * ynu
            xnd = (k0 + k2 * pow(r, 2) + k3 * pow(r, 3) + k4 * pow(r, 4)) * xnu
            ypd = f_y * ynd + 240
            xpd = f_x * xnd + 320
            undistX[i, j] = xpd
            undistY[i, j] = ypd

    return undistX, undistY
def undistort(img,undistX,undistY):
    return cv2.remap(img,undistX,undistY,cv2.INTER_LINEAR)



undistX, undistY = get_undistparams()

cam = cv2.VideoCapture(0)
ret, prev = cam.read()
prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
prevgray = cv2.resize(prevgray, dsize=(640, 480))

prevgray = undistort(prevgray,undistX,undistY)


ret, prev = cam.read()
prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
prevgray = cv2.resize(prevgray, dsize=(640, 480))
prevgray = undistort(prevgray,undistX,undistY)
cv2.imshow("undistorted",prev)
cv2.waitKey(10000)
    