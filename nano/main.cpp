/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>

#include "Error.h"
#include "Thread.h"

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <NvEglRenderer.h>
#include <NvJpegEncoder.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <signal.h>

#include <sys/socket.h>
#include <sys/un.h>
#define SOCKET_NAME "/tmp/flower.socket"



#define SPECKLE_RANGE 64
#define SPECKLE_WINDOW 5
#define TEXTURE_THRESHOLD 15
#define MIN_DISPARITY 0


using namespace std;
using namespace Argus;
using namespace EGLStream;
using namespace cv;

/* Configurations below can be overrided by cmdline */
static uint32_t CAPTURE_TIME  = 30; /* In seconds. */
static int      CAPTURE_FPS   = 21;
static uint32_t SENSOR_MODE   = 0;
static Size2D<uint32_t> PREVIEW_SIZE (960, 616);
static Size2D<uint32_t> CAPTURE_SIZE (3264, 2464);
static bool    DO_STAT = false;
static bool    VERBOSE_ENABLE = false;
static bool    DO_JPEG_ENCODE = false;

//global image frames
cv::Mat g_img1;
cv::Mat g_img2;

//synchronization stuff
bool g_img1_football1;
bool g_img1_football2;
int g_signal = 0;


#define JPEG_BUFFER_SIZE    (CAPTURE_SIZE.area() * 3 / 2)

/* Debug print macros. */
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

cv::SimpleBlobDetector::Params getBlobDetectorParams(){
    // Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;

	// Change thresholds
	params.minThreshold = 10;
	params.maxThreshold = 200;

	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 100;
    params.maxArea = 1000;
    params.minRepeatability = 2;
	/*
    // Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.1;

	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.87;

	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.01;
    */
    return params;
}

class CalibData 
{
public:
    CalibData(string filename){
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        readData(fs);
        fs.release();
    }
    void readData(FileStorage & fs) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
    }
public:
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

};

int initSocket(){
    struct sockaddr_un name;
    int connection_socket;
    int ret;

    connection_socket = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if(connection_socket == -1) std::cout << "Failed Socket Creation" << std::endl;
    memset(&name, 0, sizeof(name));

    name.sun_family = AF_UNIX;
    strncpy(name.sun_path, SOCKET_NAME, sizeof(name.sun_path) - 1);
    ret = bind(connection_socket, (const struct sockaddr *) &name, sizeof(name));
    if(ret == -1) std::cout << "Failed Socket Bind" << std::endl;
    ret = listen(connection_socket, 20);
    if(ret == -1) std::cout << "Failed Socket Listen" << std::endl;

    return connection_socket; 
}


namespace ArgusSamples
{

/*******************************************************************************
 * Base Consumer thread:
 *   Creates an EGLStream::FrameConsumer object to read frames from the
 *   OutputStream, then creates/populates an NvBuffer (dmabuf) from the frames
 *   to be processed by processV4L2Fd.
 ******************************************************************************/
class ConsumerThread : public Thread
{
public:
    explicit ConsumerThread(OutputStream* stream) :
        m_stream(stream),
        m_dmabuf(-1)
    {
    }
    virtual ~ConsumerThread();

protected:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    virtual bool processV4L2Fd(int32_t fd, uint64_t frameNumber) = 0;

    OutputStream* m_stream;
    UniqueObj<FrameConsumer> m_consumer;
    int m_dmabuf;
};

ConsumerThread::~ConsumerThread()
{
    if (m_dmabuf != -1)
        NvBufferDestroy(m_dmabuf);
}

bool ConsumerThread::threadInitialize()
{
    /* Create the FrameConsumer. */
    m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
    if (!m_consumer)
        ORIGINATE_ERROR("Failed to create FrameConsumer");

    return true;
}

bool ConsumerThread::threadExecute()
{
    IEGLOutputStream *iEglOutputStream = interface_cast<IEGLOutputStream>(m_stream);
    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);

    /* Wait until the producer has connected to the stream. */
    CONSUMER_PRINT("Waiting until producer is connected...\n");
    if (iEglOutputStream->waitUntilConnected() != STATUS_OK)
        ORIGINATE_ERROR("Stream failed to connect.");
    CONSUMER_PRINT("Producer has connected; continuing.\n");

    while (true)
    {
        /* Acquire a frame. */
        UniqueObj<Frame> frame(iFrameConsumer->acquireFrame());
        IFrame *iFrame = interface_cast<IFrame>(frame);
        if (!iFrame)
            break;

        /* Get the IImageNativeBuffer extension interface. */
        NV::IImageNativeBuffer *iNativeBuffer =
            interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
        if (!iNativeBuffer)
            ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");

        /* If we don't already have a buffer, create one from this image.
           Otherwise, just blit to our buffer. */
        if (m_dmabuf == -1)
        {
            m_dmabuf = iNativeBuffer->createNvBuffer(iEglOutputStream->getResolution(),
                                                     NvBufferColorFormat_ABGR32,
                                                     NvBufferLayout_Pitch);
            if (m_dmabuf == -1)
                CONSUMER_PRINT("\tFailed to create NvBuffer\n");
        }
        else if (iNativeBuffer->copyToNvBuffer(m_dmabuf) != STATUS_OK)
        {
            ORIGINATE_ERROR("Failed to copy frame to NvBuffer.");
        }

        /* Process frame. */
        processV4L2Fd(m_dmabuf, iFrame->getNumber());
    }

    CONSUMER_PRINT("Done.\n");

    requestShutdown();

    return true;
}

bool ConsumerThread::threadShutdown()
{
    return true;
}

/*******************************************************************************
 * Preview Consumer thread:
 *   Read frames from the OutputStream and render it on display.
 ******************************************************************************/
class PreviewConsumerThread : public ConsumerThread
{
public:
    int id;
    bool * football;
    cv::Mat * mat_ptr;
    PreviewConsumerThread(OutputStream *stream, int i);
    ~PreviewConsumerThread();

private:
    bool threadInitialize();
    bool threadShutdown();
    bool processV4L2Fd(int32_t fd, uint64_t frameNumber);
};

PreviewConsumerThread::PreviewConsumerThread(OutputStream *stream, int i) :
    ConsumerThread(stream)
{
    id = i;
    if(id == 1){
        mat_ptr = & g_img1;
        football = & g_img1_football1;
    }
    else {
        mat_ptr = & g_img2;
        football = & g_img1_football2;
    }
}

PreviewConsumerThread::~PreviewConsumerThread()
{
}

bool PreviewConsumerThread::threadInitialize()
{
    if (!ConsumerThread::threadInitialize())
        return false;

    return true;
}

bool PreviewConsumerThread::threadShutdown()
{
    return ConsumerThread::threadShutdown();
}

bool PreviewConsumerThread::processV4L2Fd(int32_t fd, uint64_t frameNumber)
{
    if(!*football){
        void *pdata = NULL;
        NvBufferMemMap(fd, 0, NvBufferMem_Read, &pdata);
        NvBufferMemSyncForCpu(fd, 0, &pdata);

        cv::Mat imgbuf = cv::Mat(PREVIEW_SIZE.height(),
                                PREVIEW_SIZE.width(),
                                CV_8UC4, pdata);
        //cv::Mat display_img;
        cvtColor(imgbuf, * mat_ptr, 3); //3 replaced the weird constant CV_RGBA2BGR
        // cvtColor(imgbuf, g_img1, 3); //3 replaced the weird constant CV_RGBA2BGR

        NvBufferMemUnMap(fd, 0, &pdata);
        //cout << "Found stream " << streamName << endl;
        //cv::imshow(to_string(id), * mat_ptr);
        //cv::waitKey(1);
        //usleep(10);
        * football = 1;
    }
    return true;

}
void updateFrames() {
    cv::Mat frame_1, frame_2, hsv_1, hsv_2, maskYellow_1, maskYellow_2, maskWhite_1, maskWhite_2, 
    maskYellow_blurred_1, maskYellow_blurred_2, maskWhite_blurred_1, maskWhite_blurred_2,
    invBlurredMask, blurred;
    cv::Size maskDim, kernelSize;
    cv::SimpleBlobDetector::Params params = getBlobDetectorParams();
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    int key;

	// Storage for blobs
	std::vector<cv::KeyPoint> keypoints;
    if(true){
        while(!g_img1_football1 || !g_img1_football2){}
        // frame = g_img1;
        // cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // //add some yellow
        // cv::inRange(hsv, cv::Scalar(14,64,120), cv::Scalar(30,255,255), maskYellow);
        // //blur
        // maskDim = maskYellow.size();
        // kernelSize.height = maskDim.height / (75);
        // kernelSize.width = maskDim.width / (75);
        // cv:blur(maskYellow, maskYellow_blurred, kernelSize);
        // ones = cv::Mat::ones(maskYellow_blurred.size(), maskYellow_blurred.type());
        //cv::bitwise_and(maskYellow_blurred, ones, invBlurredMask);
        //cv::bitwise_not(maskYellow_blurred, invBlurredMask);
        //cv::blur(invBlurredMask, blurred, kernelSize);
        //detector->detect(blurred, keypoints);
        //cout << keypoints.size() << endl; 
        //cv::drawKeypoints(frame, keypoints, frame, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        //Blob
        //cv::imshow("Final", frame);
        //cv::imshow("blurred", blurred);
        //cv::imshow("Yellow Mask", maskYellow);
        // Get the pressed value
        // cv::imshow("raw1", g_img1);
        // cv::imshow("raw2", g_img2);
        // key = (cv::waitKey(1) & 0xFF);
        // if(key == 'q'){
        //     break;
        // }
        cv::cvtColor(g_img1, frame_1, cv::COLOR_BGR2HSV);
        cv::cvtColor(g_img2, frame_2, cv::COLOR_BGR2HSV);

        // cv::inRange(frame_1, cv::Scalar(14,64,120), cv::Scalar(30,255,255), maskYellow_1);
        // cv::inRange(frame_2, cv::Scalar(14,64,120), cv::Scalar(30,255,255), maskYellow_2);

        cv::inRange(frame_1, cv::Scalar(0,0,230), cv::Scalar(179, 15, 255), maskWhite_1); //TODO: Calibrate later?
        cv::inRange(frame_2, cv::Scalar(0,0,230), cv::Scalar(179, 15, 255), maskWhite_2);
        // cv::cvtColor(maskWhite_1, maskWhite_1, cv::COLOR_GRAY2BGR);
        // cv::cvtColor(maskWhite_1, maskWhite_1, cv::COLOR_BGR2HSV);
        // cv::cvtColor(maskWhite_2, maskWhite_2, cv::COLOR_GRAY2BGR);
        // cv::cvtColor(maskWhite_2, maskWhite_2, cv::COLOR_BGR2HSV);


        // // cv::Mat ones_1 = cv::Mat::ones(maskWhite_1.size(), maskWhite_1.type());
        // // cv::Mat ones_2 = cv::Mat::ones(maskWhite_2.size(), maskWhite_2.type());
        // // ones_1 = maskWhite & ones_1;
        // // ones_2 = maskWhite & 

        // cv::bitwise_and(frame_1, maskWhite_1, frame_1);
        // cv::bitwise_and(frame_2, maskWhite_2, frame_2);
        // cv::bitwise_or(maskWhite_1, maskYellow_1, maskWhite_1);
        // cv::bitwise_or(maskWhite_2, maskYellow_2, maskWhite_2);
        cv::cvtColor(maskWhite_1, g_img1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(maskWhite_2, g_img2, cv::COLOR_GRAY2BGR);

        g_img1_football1 = 0;
        g_img1_football2 = 0;
    }

}

static void saveXYZ(const char* filename, const cv::Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

cv::Vec3f getStereo( int alg, int SADWindowSize, int numberOfDisparities, bool no_display, bool color_display, float scale,
 string intrinsic_filename, string extrinsic_filename, string disparity_filename, string point_cloud_filename,Ptr<StereoBM> bm, 
 Ptr<StereoSGBM> sgbm){

    ArgusSamples::updateFrames();

    using namespace cv;

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4, STEREO_HH4=5 };

    int color_mode = alg == STEREO_BM ? 0 : -1;
    // Mat img1 = imread(img1_filename, color_mode);
    // Mat img2 = imread(img2_filename, color_mode);
    Mat img1, img2;
    if(alg == STEREO_BM){
        cv::cvtColor(g_img1, img1, cv::COLOR_RGBA2GRAY);
        cv::cvtColor(g_img2, img2, cv::COLOR_RGBA2GRAY);
    }
    else {
        img1 = g_img1;
        img2 = g_img2;
    }

    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }

    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;

    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;
    }

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(TEXTURE_THRESHOLD);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(SPECKLE_WINDOW);
    bm->setSpeckleRange(SPECKLE_RANGE);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(MIN_DISPARITY);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(SPECKLE_WINDOW);
    sgbm->setSpeckleRange(SPECKLE_RANGE);
    sgbm->setDisp12MaxDiff(1);
    if(alg==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_HH4)
        sgbm->setMode(StereoSGBM::MODE_HH4);
    else if(alg==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    float disparity_multiplier = 1.0f;
    if( alg == STEREO_BM )
    {
        bm->compute(img1, img2, disp);
        if (disp.type() == CV_16S)
            disparity_multiplier = 16.0f;
    }
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_HH4 || alg == STEREO_3WAY )
    {
        sgbm->compute(img1, img2, disp);
        if (disp.type() == CV_16S)
            disparity_multiplier = 16.0f;
    }
    t = getTickCount() - t;
    //printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);

    Mat disp8_3c;
    if (color_display)
        cv::applyColorMap(disp8, disp8_3c, 1);

    if(!disparity_filename.empty())
        imwrite(disparity_filename, color_display ? disp8_3c : disp8);

    if(!point_cloud_filename.empty())
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        Mat floatDisp;
        disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
        reprojectImageTo3D(floatDisp, xyz, Q, true);
        saveXYZ(point_cloud_filename.c_str(), xyz);
        printf("\n");

        return 2;
    }

    std::ostringstream oss;
    oss << "disparity  " << (alg==STEREO_BM ? "bm" :
                                alg==STEREO_SGBM ? "sgbm" :
                                alg==STEREO_HH ? "hh" :
                                alg==STEREO_VAR ? "var" :
                                alg==STEREO_HH4 ? "hh4" :
                                alg==STEREO_3WAY ? "sgbm3way" : "");
    oss << "  blocksize:" << (alg==STEREO_BM ? SADWindowSize : sgbmWinSize);
    oss << "  max-disparity:" << numberOfDisparities;
    std::string disp_name = oss.str();


    //blob 
    Mat blobs;
    inRange(disp8, Scalar(1), Scalar(255), blobs);
    GaussianBlur(blobs, blobs, Size(0,0), 11, 0); //TODO: Calibrate
    /*std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(blobs, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<vector<Point>> hull(contours.size());
    for(size_t i = 0; i < contours.size(); i++){
        convexHull(contours[i], hull[i]);
    }

    Moments m;
    Moments M;
    int maxI;
    int maxArea = 0;
    std::vector<Point> maxContour;
    for(size_t i = 0; i < contours.size(); i++){
        m = moments(contours[(int) i]);
        if(m.m00 > maxArea){
            maxArea = m.m00;
            maxI = i;
            maxContour = contours[(int) i];
            M = m;
        }
    }
    
    Point center  = Point(int(M.m10/M.m00), int(M.m01/M.m00));
    drawContours(blobs, hull, maxI, Scalar(255));
    drawContours(mask, hull, maxI, 255, -1); */

    Mat mask = Mat::zeros(disp8.size(), disp8.type());
    double minVal, maxVal;
    Point minLoc, maxLoc;
    minMaxLoc(blobs, &minVal, &maxVal, &minLoc, &maxLoc);

    circle(mask, maxLoc, 11, Scalar(255), -1);
    circle(blobs, maxLoc, 11, Scalar(255), 2);
    //circle(disp8, maxLoc, 11, Scalar(255), 2);

    minMaxLoc(disp8, &minVal, &maxVal, &minLoc, &maxLoc, mask);

    circle(blobs, maxLoc, 3, Scalar(255), 2);
    circle(disp8, maxLoc, 3, Scalar(255), 2);



    //experimental:
    Mat xyz;
    Mat floatDisp;
    disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
    reprojectImageTo3D(floatDisp, xyz, Q, true);

    cv::Vec3f elem = xyz.at<cv::Vec3f>(maxLoc);
    //std::cout << elem << " D = " << sqrt(elem[0]*elem[0] + elem[1]*elem[1] + elem[2]*elem[2]) << std::endl; 
    //printf("press ESC key or CTRL+C to close...");
    fflush(stdout);
    //printf("\n");

    if(!no_display){

        namedWindow("left", cv::WINDOW_NORMAL);
        imshow("left", img1);
        namedWindow("right", cv::WINDOW_NORMAL);
        imshow("right", img2);
        imshow("blobs", blobs);
        namedWindow(disp_name, cv::WINDOW_AUTOSIZE);
        imshow(disp_name, color_display ? disp8_3c : disp8);
    }

    return elem;
}
/**
 * Argus Producer thread:
 *   Opens the Argus camera driver, creates two OutputStreams to output to
 *   Preview Consumer and Capture Consumer respectively, then performs repeating
 *   capture requests for CAPTURE_TIME seconds before closing the producer and
 *   Argus driver.
 *
 * @param renderer     : render handler for camera preview
 */

}; /* namespace ArgusSamples */

static void print_help(char** argv)
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: %s <left_image> <right_image> [--algorithm=bm|sgbm|hh|hh4|sgbm3way] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [--scale=scale_factor>] [-i=<intrinsic_filename>] [-e=<extrinsic_filename>]\n"
           "[--no-display] [--color] [-o=<disparity_image>] [-p=<point_cloud_file>]\n", argv[0]);
}

void signalHandler(int signal){
    g_signal = signal;
}

int main(int argc, char** argv)
{
    signal(SIGINT, signalHandler);
    using namespace ArgusSamples;
    // UniqueObj<OutputStream> captureStream;
    //CaptureConsumerThread *captureConsumerThread = NULL;
    g_img1_football1 = 0;
    g_img1_football2 = 0;

    /* Create the CameraProvider object and get the core interface */
    UniqueObj<CameraProvider> cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
    if (!iCameraProvider)
        ORIGINATE_ERROR("Failed to create CameraProvider");

    /* Get the camera devices */
    std::vector<CameraDevice*> cameraDevices;
    iCameraProvider->getCameraDevices(&cameraDevices);
    if (cameraDevices.size() == 0)
        ORIGINATE_ERROR("No cameras available");
    cout << "found " << cameraDevices.size() << " camera devices" << endl;
    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(cameraDevices[0]);
    if (!iCameraProperties)
        ORIGINATE_ERROR("Failed to get ICameraProperties interface");
    ICameraProperties * iCameraProperties_2 = interface_cast<ICameraProperties>(cameraDevices[1]);
    if (!iCameraProperties_2)
        ORIGINATE_ERROR("Failed to get ICameraProperties interface");

    /* Create the capture session using the first device and get the core interface */
    UniqueObj<CaptureSession> captureSession(
            iCameraProvider->createCaptureSession(cameraDevices[0]));
    UniqueObj<CaptureSession> captureSession_2(
        iCameraProvider->createCaptureSession(cameraDevices[1]));

    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
    ICaptureSession *iCaptureSession_2 = interface_cast<ICaptureSession>(captureSession_2);
    if (!iCaptureSession)
        ORIGINATE_ERROR("Failed to get ICaptureSession interface");
    if (!iCaptureSession_2)
        ORIGINATE_ERROR("Failed to get ICaptureSession interface");

    /* Initiaialize the settings of output stream */
    PRODUCER_PRINT("Creating output streams\n");
    UniqueObj<OutputStreamSettings> streamSettings(
        iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    UniqueObj<OutputStreamSettings> streamSettings_2(
        iCaptureSession_2->createOutputStreamSettings(STREAM_TYPE_EGL));
    IEGLOutputStreamSettings *iEglStreamSettings =
        interface_cast<IEGLOutputStreamSettings>(streamSettings);
    IEGLOutputStreamSettings *iEglStreamSettings_2 =
        interface_cast<IEGLOutputStreamSettings>(streamSettings_2);
    if (!iEglStreamSettings)
        ORIGINATE_ERROR("Failed to get IEGLOutputStreamSettings interface");
    if (!iEglStreamSettings_2)
        ORIGINATE_ERROR("Failed to get IEGLOutputStreamSettings interface");

    iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEglStreamSettings->setResolution(PREVIEW_SIZE);

    iEglStreamSettings_2->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEglStreamSettings_2->setResolution(PREVIEW_SIZE);

    /* Based on above streamSettings, create the preview stream,
       and capture stream if JPEG Encode is required */
    UniqueObj<OutputStream> previewStream(iCaptureSession->createOutputStream(streamSettings.get()));
    UniqueObj<OutputStream> previewStream_2(iCaptureSession_2->createOutputStream(streamSettings_2.get()));

    /* Launch the FrameConsumer thread to consume frames from the OutputStream */
    PRODUCER_PRINT("Launching consumer threads\n");

    PreviewConsumerThread previewConsumerThread_2(previewStream_2.get(), 2);
    PROPAGATE_ERROR(previewConsumerThread_2.initialize());
    PreviewConsumerThread previewConsumerThread(previewStream.get(), 1);
    PROPAGATE_ERROR(previewConsumerThread.initialize());

    /* Wanit until the consumer thread is conected to the stream */
    PROPAGATE_ERROR(previewConsumerThread_2.waitRunning());
    PROPAGATE_ERROR(previewConsumerThread.waitRunning());



    /* Create capture request and enable its output stream */
    UniqueObj<Request> request(iCaptureSession->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(request);
    if (!iRequest)
        ORIGINATE_ERROR("Failed to create Request");
    iRequest->enableOutputStream(previewStream.get());

    UniqueObj<Request> request_2(iCaptureSession_2->createRequest());
    IRequest *iRequest_2 = interface_cast<IRequest>(request_2);
    if (!iRequest_2)
        ORIGINATE_ERROR("Failed to create Request_2");
    iRequest_2->enableOutputStream(previewStream_2.get());


    ISensorMode *iSensorMode;
    std::vector<SensorMode*> sensorModes;
    iCameraProperties->getBasicSensorModes(&sensorModes);
    if (sensorModes.size() == 0)
        ORIGINATE_ERROR("Failed to get sensor modes");
    ISensorMode *iSensorMode_2;
    std::vector<SensorMode*> sensorModes_2;
    iCameraProperties_2->getBasicSensorModes(&sensorModes_2);
    if (sensorModes_2.size() == 0)
        ORIGINATE_ERROR("Failed to get sensor modes _2");

    PRODUCER_PRINT("Available Sensor modes :\n");
    for (uint32_t i = 0; i < sensorModes.size(); i++) {
        iSensorMode = interface_cast<ISensorMode>(sensorModes[i]);
        Size2D<uint32_t> resolution = iSensorMode->getResolution();
        PRODUCER_PRINT("[%u] W=%u H=%u\n", i, resolution.width(), resolution.height());
    }

    ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    ISourceSettings *iSourceSettings_2 = interface_cast<ISourceSettings>(iRequest_2->getSourceSettings());
    if (!iSourceSettings)
        ORIGINATE_ERROR("Failed to get ISourceSettings interface");
    if (!iSourceSettings_2)
        ORIGINATE_ERROR("Failed to get ISourceSettings interface");

    /* Check and set sensor mode */
    if (SENSOR_MODE >= sensorModes.size())
        ORIGINATE_ERROR("Sensor mode index is out of range");
    SensorMode *sensorMode = sensorModes[0];
    iSensorMode = interface_cast<ISensorMode>(sensorMode);
    SensorMode *sensorMode_2 = sensorModes_2[1];
    iSensorMode = interface_cast<ISensorMode>(sensorMode_2);
    //iSourceSettings->setSensorMode(sensorMode); // maybe add these back
    //iSourceSettings_2->setSensorMode(sensorMode_2);

    /* Check fps */
    Argus::Range<uint64_t> sensorDuration(iSensorMode->getFrameDurationRange());
    Argus::Range<uint64_t> desireDuration(1e9/CAPTURE_FPS+0.9);
    if (desireDuration.min() < sensorDuration.min() ||
            desireDuration.max() > sensorDuration.max()) {
        PRODUCER_PRINT("Requested FPS out of range. Fall back to 30\n");
        CAPTURE_FPS = 30;
    }
    /* Set the fps */
    iSourceSettings->setFrameDurationRange(Argus::Range<uint64_t>(1e9/CAPTURE_FPS));
    iSourceSettings_2->setFrameDurationRange(Argus::Range<uint64_t>(1e9/CAPTURE_FPS));

    /* Submit capture requests. */
    PRODUCER_PRINT("Starting repeat capture requests.\n");
    if (iCaptureSession->repeat(request.get()) != STATUS_OK)
        ORIGINATE_ERROR("Failed to start repeat capture request");
    if (iCaptureSession_2->repeat(request_2.get()) != STATUS_OK)
        ORIGINATE_ERROR("Failed to start repeat capture request _2");

    /* Wait for CAPTURE_TIME seconds. */
    //sleep(CAPTURE_TIME);
    sleep(1);
/*
    //adding for capturing image frames:
    int counter = 0;
    int key;
        while(1)
        {
            updateFrames();
            imshow("left", g_img1);
            imshow("right", g_img2);
            key = (waitKey(50) & 0xFF);
            if(key == 'q')
                break;
            else if(key == 'w'){
                imwrite("left_" + to_string(counter) + ".png", g_img1);
                imwrite("right_" + to_string(counter++) + ".png", g_img2);
                std::cout << "Wrote " + to_string(counter - 1) + " pair" << std::endl;
            }
        }
*/

    //OPENCV PORTION
    using namespace cv;

    //CalibData calibCam0 = CalibData("out_camera_0_data.xml");
    //CalibData calibCam1 = CalibData("out_camera_1_data.xml");

    std::string img1_filename = "";
    std::string img2_filename = "";
    std::string intrinsic_filename = "";
    std::string extrinsic_filename = "";
    std::string disparity_filename = "";
    std::string point_cloud_filename = "";

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4, STEREO_HH4=5 };
    int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    bool color_display;
    float scale;

    Ptr<StereoBM> bm = cv::cuda::StereoBM::create(16,9); //make this the cuda version
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    cv::CommandLineParser parser(argc, argv,
        "{@arg1||}{@arg2||}{help h||}{algorithm||}{max-disparity|0|}{blocksize|0|}{no-display||}{color||}{scale|1|}{i||}{e||}{o||}{p||}");
    if(parser.has("help")){
        print_help(argv);
        return 0;
    }
    img1_filename = samples::findFile(parser.get<std::string>(0)); //probably wont need these
    img2_filename = samples::findFile(parser.get<std::string>(1));
    if (parser.has("algorithm")) //start parsing the command line
    {
        std::string _alg = parser.get<std::string>("algorithm");
        alg = _alg == "bm" ? STEREO_BM :
            _alg == "sgbm" ? STEREO_SGBM :
            _alg == "hh" ? STEREO_HH :
            _alg == "var" ? STEREO_VAR :
            _alg == "hh4" ? STEREO_HH4 :
            _alg == "sgbm3way" ? STEREO_3WAY : -1;
    }
    numberOfDisparities = parser.get<int>("max-disparity");



    SADWindowSize = parser.get<int>("blocksize");
    scale = parser.get<float>("scale");
    no_display = parser.has("no-display");
    color_display = parser.has("color");
    if( parser.has("i") )
        intrinsic_filename = parser.get<std::string>("i");
    if( parser.has("e") )
        extrinsic_filename = parser.get<std::string>("e");
    if( parser.has("o") )
        disparity_filename = parser.get<std::string>("o");
    if( parser.has("p") )
        point_cloud_filename = parser.get<std::string>("p");
    if (!parser.check())
    {
        parser.printErrors();
        return 1;
    }
    if( alg < 0 )
    {
        printf("Command-line parameter error: Unknown stereo algorithm\n\n");
        print_help(argv);
        return -1;
    }
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
    {
        printf("Command-line parameter error: The max disparity (--max-disparity=<...>) must be a positive integer divisible by 16\n");
        print_help(argv);
        return -1;
    }
    if (scale < 0)
    {
        printf("Command-line parameter error: The scale factor (--scale=<...>) must be a positive floating-point number\n");
        return -1;
    }
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
        return -1;
    }
    if( img1_filename.empty() || img2_filename.empty() )
    {
        printf("Command-line parameter error: both left and right images must be specified\n");
        return -1;
    }
    if( (!intrinsic_filename.empty()) ^ (!extrinsic_filename.empty()) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename.empty() && !point_cloud_filename.empty() )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

    //init Socket
    int connection_socket = initSocket();
    int data_socket;
    int result;
    char buff[12];
    char readBuff[10];
    int size = 0;
    fd_set set;
    timeval tv;
    tv.tv_sec = 100;
    tv.tv_usec = 0;

    FD_ZERO(&set);
    FD_SET(data_socket, &set);
    data_socket = accept(connection_socket, NULL, NULL);
    if(data_socket == -1) std::cout << "Failed Accept Socket" << std::endl;

    cv::Vec3f loc;
    while(1)
    {
        loc = ArgusSamples::getStereo(alg, SADWindowSize, numberOfDisparities, no_display, color_display, scale, intrinsic_filename, extrinsic_filename, disparity_filename, point_cloud_filename, bm, sgbm);
        if((waitKey(20) & 0xFF) == 'q') //ESC (prevents closing on actions like taking screenshots)
            break;
        memset(buff, 0, sizeof(buff));
        for(int i = 0; i < 3; i++){
            std::memcpy(&buff[i*4], &loc[i], 4);
        }
        //check for signals to exit
        if(g_signal != 0){
            std::cout << "Closing Resources!" << std::endl;
            break;
        }
        write(data_socket, buff, sizeof(buff));
        //send the location data via the socket
    }
    shutdown(data_socket, 2);
    close(data_socket);
    unlink(SOCKET_NAME);
    

    /* Stop the repeating request and wait for idle. */
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();
    iCaptureSession_2->stopRepeat();
    iCaptureSession_2->waitForIdle();

    /* Destroy the output stream to end the consumer thread. */
    previewStream.reset();
    previewStream_2.reset();

    /* Wait for the consumer thread to complete. */
    PROPAGATE_ERROR(previewConsumerThread.shutdown());
    PROPAGATE_ERROR(previewConsumerThread_2.shutdown());

    PRODUCER_PRINT("Done -- exiting.\n");
    return 0;
}
