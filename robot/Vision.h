/**
 * @file        Vision.h
 * @brief       Functions to manage and control the vision (from the Kinect).
 */
#ifndef VISION_H
#define VISION_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
// openni
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>
// opencv
#include <cv.h>
#include <highgui.h>


/*-------------------- DEFINES  --------------------*/
#define VISION_XML_CONFIG_PATH       "../../Config/SamplesConfig.xml"
#define VISION_XML_CONFIG_PATH_LOCAL "/home/jon/kinect/project/SamplesConfig.xml"
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define DEPTH_STREAM
#define COMPRESSION_QUALITY 95    // quality of streamed images. can be 0-100 with a higher number representing a larger file size but a higher quality image.
//#define COLOR_STREAM
#if COLOR_STREAM
    #error "NO COLOUR STREAM PLESE :(... or write it yourself"
#endif


/*--------------------  MACROS  --------------------*/
// None.


/*---------------- CLASS DEFINITION ----------------*/
using namespace xn; // GET RID OF THIS LINE!


class Vision
{
public:
    Vision (void);
    ~Vision (void);
    
    void captureFrame (void);
    void streamFrame  (void);
    

private:
    void loadCameraConfiguration   (void);
    void initialiseCamera          (void);
    void shutdownCamera            (void);
    void setupServers              (void);
    void setupServer               (void);
    void checkForClientConnections (void);
    
    void   createColourDepthImage (cv::Mat* dst, uint16_t* src);
    XnBool fileExists             (const char *fn);
    
    // Vision Variables
    Context         mContext;
    ScriptNode      mScriptNode;
    // OpenNI image containers
    DepthGenerator  mDepthGenerator;
    ImageGenerator  mColorGenerator;
    DepthMetaData   mDepthMetaData;
    ImageMetaData   mColorMetaData;
    const uint16_t* mDepthData;
    //const uint8_t*  mColorData;
    XnFPSData      mXnFPS;
    // OpenCV image containers
    cv::Mat              mStreamingDepthRaw(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    std::vector<uint8_t> mStreamingDepthJPEG;
    // Streaming server variables
    uint8_t mClientConnected;
};

#endif
