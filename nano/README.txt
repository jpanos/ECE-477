This is the image processing module for ECE477 Senior Design Group 4 - Sowin Seeds -
Purdue University Fall 2020

Description of Files:
main.cpp:
	Bulk of the image processing. For now, the command used to run this file is in "command.txt".
	This is a temporary implementation and only for debugging purposes. main.cpp opens a socket
	in /var/flowers.socket and streams x,y,z coordinates of the centroid of the predicted 
	flower group.

uart_mod.py:
	This is the python file that handles basic processing and uart communication between the
	Jetson Nano and the STM32H745.

command.txt:
	The command used to run main.cpp when it is compiled with ./make

Makefile:
	run this to compile main.cpp

stereo_capture.cpp:
	This assists in capturing images from stereo CSI cameras

intrinsics.yml & extrinsics.yml:
	Calibration data for our CSI cameras.

stereo_calib.cpp:
	This is not written or owned by me. See the header of the file for more details.
	
