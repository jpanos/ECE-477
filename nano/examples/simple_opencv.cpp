#include <stdio.h>
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
  VideoCapture cap("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720,format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw,format=I420 ! appsink");

  if (!cap.isOpened())
    {
      cout << "Failed to open camera." << endl;
      return -1;
    }

  for(;;)
    {
      Mat frame;
      cap >> frame;
      Mat bgr;
      cvtColor(frame, bgr, CV_YUV2BGR_I420);
      imshow("original", bgr);
      waitKey(1);
    }

  cap.release();
}
