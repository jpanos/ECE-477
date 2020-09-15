# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

# import libraries
import cv2
import numpy as np

# Open Image in Grayscale
def openImg(imgNum) :
	imgNumStr = str(imgNum)
	filename = "/home/jackie/Documents/flowerPictures/blossoms" + imgNumStr + ".jpg" # todo: blossoms 7 needs to be jpg not png
	img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE) # read the image
	cv2.imshow('win',img)
	cv2.waitKey(40)
	return (img)

# Open Image in Color (for color masking)
def openImgColor(imgNum) :
	imgNumStr = str(imgNum)
	filename = "/home/jackie/Documents/flowerPictures/blossoms" + imgNumStr + ".jpg"  # todo: blossoms 7 needs to be jpg not png
	img = cv2.imread(filename,cv2.IMREAD_COLOR)
	#cv2.imshow('win3', img)
	#cv2.waitKey(40)
	return (img)

# Color Blob detector params
def getBlobDetectParams():
	params = cv2.SimpleBlobDetector_Params()
	params.minThreshold = 10
	params.maxThreshold = 200
	# todo: add more params
	return params

# Main
def main():
	imageNumbers = range(14) # number of images in folder
	params = getBlobDetectParams()
	detector = cv2.SimpleBlobDetector_create(params)
	# basic color blob loop
	"""  
	for x in imageNumbers:
		print("Printing image %d\n" % (x+1))
		img = openImg(x+1)
		keypoints = detector.detect(img)
		outImg = cv2.drawKeypoints(img, keypoints, img,(255, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		cv2.imshow('win2',outImg)
		cv2.waitKey(0)
	return
	"""
	# color masking
	for x in imageNumbers:
		print("Printing image %d\n" % (x+1))
		img = openImgColor(x+1)
		# convert bgr color space of image to hsv
		hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
		darkerThr = np.array([0,0,191]) # H: 0-179, 0-255, V: 0-255
		lighterThr = np.array([179, 79,255])
		#mask
		mask = cv2.inRange(hsv, darkerThr, lighterThr)
		# filter out green
		"""
		greenLow = np.array([45, 79,0])
		greenHigh = np.array([127,15,255])
		mask2 = cv2.inRange(hsv, greenLow,greenHigh)
		mask = mask - mask2
		"""
		# filter out all blue
		blueLow = np.array([85,51,0])
		bluehigh = np.array([122,255,255])
		maskblue = cv2.inRange(hsv, blueLow,bluehigh)
		maskblue = 255-maskblue
		# add some more yellow
		yellowLow = np.array([14,64,120])
		yellowHigh = np.array([30,255,255])
		maskYellow = cv2.inRange(hsv, yellowLow, yellowHigh)
		# multiply image with mask (could try or also)
		mask_combined = mask | maskYellow
		result = cv2.bitwise_and(img, img, mask= mask_combined)
		result = cv2.bitwise_and(result, result, mask = maskblue) # cut out the blue
		stamenOnly = cv2.bitwise_and(img,img, mask=maskYellow)
		# todo: track stamen blobs
		# 1. blur
		maskDim = np.shape([mask])
		kernalSize = (int(maskDim[1]/75), int(maskDim[2]/75)) # todo calibrate denom on nano
		blurredMask = cv2.blur(maskYellow,kernalSize)
	#	loResblurredMask = (blurredMask & 1) * 255
		blurredPetalMask = cv2.blur(mask,kernalSize)
	#	loResBlurredPetal = (blurredPetalMask & 1) * 255
		# convolve the two masks, the regions where highest convolution with offset = petal
		muls = blurredMask * blurredPetalMask
		muls = (muls / 255)
		# display
		cv2.imshow('img', img)
		cv2.imshow('mask',mask_combined)
		cv2.imshow('result', result)
		cv2.imshow('stamen', stamenOnly)
		cv2.imshow('blurredStamen',blurredMask)
	#	cv2.imshow('lowResBlurStamen', loResblurredMask)
		cv2.imshow('blurredPetalmask', blurredPetalMask)
	#	cv2.imshow('lowResBlurredPetal',loResBlurredPetal)
		cv2.imshow('mutipliedMasks', muls)
		cv2.waitKey(0)
	return
# call main
main()