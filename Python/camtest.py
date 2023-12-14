import cv2
from cv2 import aruco
import numpy as np

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

camera = cv2.VideoCapture(0) # Initialize the camera
ret,frame = camera.read() # Take an image
grey = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY) # Make the image greyscale for ArUco detection
camera.release()
cv2.imshow('image',grey)
cv2.imwrite('aruco.png',frame)
cv2.waitKey(0) 

