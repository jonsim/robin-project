/**
 * @file        Vision.h
 * @brief       Functions to manage and control the vision (from the Kinect).
 */
#ifndef VISION_H
#define VISION_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
// openni (note warnings are suppressed (and boy are there a lot of them).
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#pragma GCC diagnostic ignored "-Wreorder"
    #include <XnOpenNI.h>
    #include <XnLog.h>
    #include <XnCppWrapper.h>
    #include <XnFPSCalculator.h>
#pragma GCC diagnostic pop
// opencv
#include <cv.h>
#include <highgui.h>
// my stuff
#include "FrameBuffer.h"
#include "Histogram.h"




/*-------------------- DEFINES  --------------------*/
#define VISION_XML_CONFIG_PATH       "CameraConfiguration.xml"
#define VISION_XML_CONFIG_PATH_LOCAL "/home/jon/individual_project/CameraConfiguration.xml"
#define DEPTH_STREAM
#define COMPRESSION_QUALITY 20    // quality of streamed images. can be 0-100 with a higher number representing a larger file size but a higher quality image.
#define MAX_DEPTH 10000           // maximum depth the kinect sensor can read. must be in the openni libraries somewhere but i cant find it.
//#define COLOR_STREAM
#if COLOR_STREAM
    #error "NO COLOUR STREAM PLESE :(... or write it yourself"
#endif

// frame buffer stuff
#define SUBSAMPLING_FACTOR  2
#define FRAME_RETENTION     5

// panic thresholding stuff
// values for full frame 640x480 res
//#define HIST_STATIC_PANIC_THRESHOLD  180000 // this many readings in the near range will cause instant panic
//#define HIST_DYNAMIC_PANIC_THRESHOLD  13000 // this many readings suddenly transitioning out of the near range will cause panic
// values for half frame 640x480 res
//#define HIST_STATIC_PANIC_THRESHOLD  100000 // this many readings in the near range will cause instant panic
//#define HIST_DYNAMIC_PANIC_THRESHOLD  14000 // this many readings suddenly transitioning out of the near range will cause panic
// values for half frame 320x240 res
//#define HIST_STATIC_PANIC_THRESHOLD  25000 // this many readings in the near range will cause instant panic
//#define HIST_DYNAMIC_PANIC_THRESHOLD  3500 // this many readings suddenly transitioning out of the near range will cause panic
// values for half frame 320x240 res with new ranges
#define HIST_STATIC_PANIC_THRESHOLD  20000 // this many readings in the near range will cause instant panic
#define HIST_DYNAMIC_PANIC_THRESHOLD  1000 // this many readings suddenly transitioning out of the near range will cause panic
// panic ranges
#define HIST_RANGE1_START 400   // range 1: 40 - 60cm
#define HIST_RANGE1_END   599
#define HIST_RANGE2_START 600   // range 2: 60 - 80cm
#define HIST_RANGE2_END   799




/*--------------------  MACROS  --------------------*/
// None.


/*---------------- CLASS DEFINITION ----------------*/
using namespace xn; // TODO GET RID OF THIS LINE!


class Vision
{
public:
    Vision (void);
    ~Vision (void);
    
    void  captureFrame  (void);
    const std::vector<uint8_t>* compressFrame (void);
    void  compressFrameToDisk (const char* filename);
    /*void  buildDepthHistogram (void);
    uint32_t queryDepthHistogram (uint16_t v);*/
    sint8_t shouldWePanic (void);
    float getFPS (void);
    

private:
    void loadCameraConfiguration   (void);
    void initialiseCamera          (void);
    void shutdownCamera            (void);
    void setupServers              (void);
    void setupServer               (void);
    void checkForClientConnections (void);
    
    void   createColourDepthImage (cv::Mat* dst, const uint16_t* src);
    XnBool fileExists             (const char *fn);
    
    // Vision Variables
    // Image containers
    FrameBuffer     mFrameBuffer;
    // OpenNI image containers
    Context         mContext;
    ScriptNode      mScriptNode;
    DepthGenerator  mDepthGenerator;
//    ImageGenerator  mColorGenerator;
    DepthMetaData   mDepthMetaData;
//    ImageMetaData   mColorMetaData;
    const uint16_t* mDepthData;
//    const uint8_t*  mColorData;
    XnFPSData       mXnFPS;
    // OpenCV image containers
    cv::Mat              mStreamingDepthRaw;
    std::vector<uint8_t> mStreamingDepthJPEG;
    // Other image algorithm datas
//    uint32_t        mDepthHistogram[MAX_DEPTH];
//    uint32_t        mLHistogramErrorRangeCount;
//    uint32_t        mLHistogramNearRangeCount;
//    uint32_t        mRHistogramErrorRangeCount;
//    uint32_t        mRHistogramNearRangeCount;
//    Histogram       mHistogram;
//    Histogram       mLHistogram;
//    Histogram       mRHistogram;
};

#endif
