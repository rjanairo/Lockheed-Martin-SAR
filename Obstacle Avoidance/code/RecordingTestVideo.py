import cv2 as cv
import numpy as np

fourcc = cv.VideoWriter_fourcc(*'XVID')
out_original = cv.VideoWriter('Testvideo.avi', fourcc, 20.0, (640, 480))

cam = cv.VideoCapture(1)
while True:
    ret, img = cam.read(0)
    out_original.write(img)

    cv.imshow("OpticalFlow", img)  # displaying image with flow on it, for illustration purposes

    key = cv.waitKey(30)
    if key == ord('q'):
        out_original.release()
        break

