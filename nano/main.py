
import cv2
import numpy as np


# Color Blob detector params
def getBlobDetectParams(): # todo: Calibrate params on NANO
	params = cv2.SimpleBlobDetector_Params()
	params.minThreshold = 10
	params.maxThreshold = 200
	params.filterByArea = 1
	params.minArea = 100
	params.maxArea = 1000
	# todo: add more params
	return params

def getShellcommand():
	gsstreamerPL = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate = 21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink'
	return gsstreamerPL

# Main
def main():
	params = getBlobDetectParams()
	detector = cv2.SimpleBlobDetector_create(params)
	command = getShellcommand()
	# begin stream
	video_capture = cv2.VideoCapture(command, cv2.CAP_GSTREAMER)
	if video_capture.isOpened(): # check to see if camera opens 
		cv2.namedWindow("Live Feed", cv2.WINDOW_AUTOSIZE)
		while 1:
			return_key, img = video_capture.read()
			if not return_key:
				break

			hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
			# add some more yellow
			yellowLow = np.array([14,64,120])
			yellowHigh = np.array([30,255,255])
			maskYellow = cv2.inRange(hsv, yellowLow, yellowHigh)
			# todo: track stamen blobs
			# 1. blur
			maskDim = np.shape([maskYellow])
			kernalSize = (int(maskDim[1]/75), int(maskDim[2]/75)) # todo calibrate denom on nano
			blurredImage = cv2.blur(img, kernalSize) # Blur the current frame
			# todo: Filter blobs better (THIS IS IT)!!!!
			invBlurredMask = ~((maskYellow & 1)*255)
			invBlurredMask = cv2.blur(invBlurredMask,kernalSize)
			keypoints = detector.detect(invBlurredMask)
			invBlurredImg = cv2.drawKeypoints(img, keypoints, img, (0, 0, 255), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			#BLOB
			cv2.imshow('FinalProduct',invBlurredImg)
			cv2.imshow('mask', invBlurredMask)
			key = cv2.waitKey(30)&0xff
			if key == 27:
				break # quit on esc key
		
		video_capture.release()
		cv2.destroyAllWindows()
	else:
		print("Unable to open camera")

	return
# call main
main()
