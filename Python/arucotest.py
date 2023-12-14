import cv2
from cv2 import aruco
import numpy as np

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

frame = cv2.imread('aruco.png') # Retrieve image
grey = cv2.cvtColor(frame,cv2.COLOR_RGB2GRAY) # Make the image greyscale for ArUco detection

corners,ids,rejected = aruco.detectMarkers(grey,aruco_dict)

overlay = cv2.cvtColor(grey,cv2.COLOR_GRAY2RGB)
overlay = aruco.drawDetectedMarkers(overlay,corners,borderColor = 4)
if(ids):
    ids = ids.flatten()
    for (outline, id) in zip(corners, ids):
        markerCorners = outline.reshape((4,2))
        overlay = cv2.putText(overlay, str(id),
                              (int(markerCorners[0,0]), int(markerCorners[0,1]) - 15),
                              cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 0, 0), 2)

cv2.imshow('image',overlay)
cv2.waitKey(0) 
cv2.destroyAllWindows()

