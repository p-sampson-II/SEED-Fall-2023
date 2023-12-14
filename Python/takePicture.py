from time import sleep
import numpy as np
import cv2

if __name__ == '__main__':
	fileName = input("File Name: ")
	
	# initialize the camera
	camera = cv2.VideoCapture(1)
	
	# Let the camera warmup
	sleep(0.1)
	
	# Get an image from the camera stream
	ret, image = camera.read()
	
	if not ret:
		print("Could not capture image from camera!")
		quit()
	else:
		# Save the image to the disk
		print("Saving image "+fileName)
		try:
			cv2.imwrite(fileName,image)
		except:
			print("Could not save "+fileName)
			pass
		
	
