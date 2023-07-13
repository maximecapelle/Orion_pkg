#!/usr/bin/env python3


import cv2

import os

# current_file_directory = os.path.dirname(os.path.abspath(__file__))
# image_path = os.path.join(current_file_directory, 'TestImage.jpg')
# print("Image path:", image_path)

# image_path = '/WS/orion_ws/src/orion_pkg/scripts/Camera'

# Load image
img = cv2.imread("TestImage.jpg")

if img is not None:
    print("Image size:", img.shape)
    cv2.imshow('TestImage', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Error: Failed to load the image")

# import cv2


# # define a video capture object
# # vid = cv2.VideoCapture(0)

# img = cv2.imread('TestImage.jpg')

# cv2.imshow('TestImage', img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# while(True):
	
# 	# Capture the video frame
# 	# by frame
# 	ret, frame = vid.read()

# 	# Display the resulting frame
# 	cv2.imshow('frame', frame)
	
# 	# the 'q' button is set as the
# 	# quitting button you may use any
# 	# desired button of your choice
# 	if cv2.waitKey(1) & 0xFF == ord('q'):
# 		break

# # After the loop release the cap object
# vid.release()
# # Destroy all the windows
# cv2.destroyAllWindows()
