# Code is found from the following repository: https://github.com/jumejume1/python-camera1/blob/main/color_thresholding.py
import cv2
import numpy as np

# Need an empty 
def nothing(pos):
	pass

cap=cv2.VideoCapture(1) # Use external camera (internal is 0)

# Create a trackbar window to configure values for blob analysis
cv2.namedWindow('Thresholds')

# Hue (colours), Saturation (colourfulness), Value (Brightness) track bar lower and upper limits
cv2.createTrackbar('LH','Thresholds',0,255, nothing)
cv2.createTrackbar('LS','Thresholds',0,255, nothing)
cv2.createTrackbar('LV','Thresholds',0,255, nothing)
cv2.createTrackbar('UH','Thresholds',255,255, nothing)
cv2.createTrackbar('US','Thresholds',255,255, nothing)
cv2.createTrackbar('UV','Thresholds',255,255, nothing)


while True:
	_,frame = cap.read()
	    
	hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) # converting the frame from BGR into HSV (hue-saturation-value)
	
	# Get lower and upper values of Hue, Saturation, and Value from trackbars
	lh=cv2.getTrackbarPos('LH','Thresholds') 
	ls=cv2.getTrackbarPos('LS','Thresholds')
	lv=cv2.getTrackbarPos('LV','Thresholds') 
	uh=cv2.getTrackbarPos('UH','Thresholds')
	us=cv2.getTrackbarPos('US','Thresholds')
	uv=cv2.getTrackbarPos('UV','Thresholds')
	
	# Defining the Range of colour into an 8-bit type array to reduce space usage
	colour_lower=np.array([lh,ls,lv],np.uint8)
	colour_upper=np.array([uh,us,uv],np.uint8)
	
	# finding the range of colour in the image
	colour=cv2.inRange(hsv,colour_lower,colour_upper)
	
	# Morphological transformation kernel  	
	kernal = np.ones((5 ,5), "uint8")

	# Perform dilation on the HSV image using the previously defined kernel
	colour = cv2.dilate(colour,kernal)

	# Display the original image next to the dilated image
	cv2.imshow("Color",colour)
	cv2.imshow("Original Image",frame)	
    
	k = cv2.waitKey(20)
	# Press the ESC key to quit 
	if k == 27:
		break

cap.release()
cv2.destroyAllWindows()