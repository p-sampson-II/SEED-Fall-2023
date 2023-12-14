import cv2
import numpy as np
from time import sleep

samples = np.empty((0,3),np.uint8)

#This function was used in tandem with the setMouseCallback function, so that whenever the mouse is being used,
#this function will be called. So every time the left click is used, it enters this function and finds where 
#the mouse is currently at and specifies which pixels are at the current location.
def left_click(event, x, y, flags, param):
        global samples
        if event == cv2.EVENT_LBUTTONDOWN: #If the left clicker is used
                print("You clicked on display pixels", x, y,", which ")
                print("Cooresponds to location ", y, x, " in image array")
                print("Pixel colors are: ")	
                b, g, r = param[y,x]
                print("RGB: [%i,%i,%i]"%(r,g,b)) 					#specify red in RGB pixel
                hsv = cv2.cvtColor(param, cv2.COLOR_RGB2HSV)	#convert to hsv color space
                h, s, v = hsv[y,x]
                sample = hsv[y,x]
                samples = np.vstack((samples,sample))
                print("HSV: [%i,%i,%i]"%(h,s,v))
                return


print("This program will print RGB and HSV values for selected pixels, and then print the minimum and maximum HSV value for all of the selected pixels.")
filename = input("Enter the filename of the image: ")
                   
print("Now displaying the image...")		#Display the image recently saved where the user specified previously
print("Once the image gets displayed, select a pixel using the mouse and we will detect the 3 RGB and 3 HSV values for that pixel")	#This prompts the user to get ready 
print("(0,0) is upper left corner. Hit and key to quit.")
img = cv2.imread(filename)			#This reads in the recently saved photo
								#the user can see the terminal at the same time
cv2.namedWindow('image')				#Name the newly openeing window 'image'
cv2.setMouseCallback('image', left_click, img)		#This line of code calls the event function where whenever the mouse is being used, it will enter the 
							#def defined above and thus reading in mouse positional values and read out RGB pixel values
cv2.imshow('image',img)			#Finally show the image and get ready for mouse GUI cases
cv2.waitKey(0)	# wait for keypress
sminmax = np.empty((3,2),np.uint8)
for ind,vec in enumerate(sminmax):
        vec[0] = np.max(samples[:,ind])
        vec[1] = np.min(samples[:,ind])
sminmax = np.transpose(sminmax)
print("Maximum and Minimum HSV values: ")
print(sminmax)
cv2.destroyAllWindows()		#Destroy all image windows 
