#include <chrono>
#include "opencv2/opencv.hpp"
#include "unistd.h"

/*
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <sstream>
*/
using namespace cv;
using namespace std;

std::string getShellCommand(int cam){
    //return "nvarguscamerasrc sensor_id=" + to_string(cam) + " ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate = 21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    return "nvarguscamerasrc sensor_id=" + to_string(cam) + " ! video/x-raw(memory:NVMM), width=3264, height=2464, format=(string)NV12, framerate = 21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char ** argv){
  // declarations
  VideoCapture cap1, cap2;
  Mat frame1, frame2;
  std::time_t t1, t2;
  int key;
  std::chrono::steady_clock::time_point begin, end;


  cout << "[DEBUG] Opening cap2" << endl;
  begin = std::chrono::steady_clock::now();
  cap2.open(getShellCommand(1), CAP_GSTREAMER);
  end = std::chrono::steady_clock::now();
  cout << "[DEBUG] Done Opening cap2 in "<< std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
  cout << "[DEBUG] Opening cap1" << endl;
  begin = std::chrono::steady_clock::now();
  cap1.open(getShellCommand(0), CAP_GSTREAMER);
  end = std::chrono::steady_clock::now();
  cout << "[DEBUG] Done Opening cap1 in " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << endl;
  
  if(cap2.isOpened() && cap2.isOpened()){
    while(true){
      cap2.read(frame2);
      imshow("frame2", frame2);
      cap1.read(frame1);
      imshow("frame1", frame1);
      key = (waitKey(1) & 0xFF);
      if(key == 'q')break;
    }
  }
  else{
    std::cout << "Failed to open Camera!" << endl;
  } 

  cap1.release();
  cap2.release();
  destroyAllWindows();


}
