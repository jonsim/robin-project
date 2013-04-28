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
#define COLOR_STREAM                // whether the color video is collected from the Kinect.
//#define COLOR_STREAMING_ENABLED     // whether the color video is streamed to connected clients.
#define DEPTH_STREAM                // whether the depth video is collected from the Kinect.
//#define DEPTH_STREAMING_ENABLED     // whether the depth video is streamed to connected clients.
#define DEPTH_COLORED               // whether the depth video is coloured before being streamed.
#define COMPRESSION_QUALITY 50      // quality of streamed images. can be 0-100 with a higher number representing a larger file size but a higher quality image.
#define MAX_DEPTH 10000             // maximum depth the kinect sensor can read. must be in the openni libraries somewhere but i cant find it.

// frame buffer stuff
#define SUBSAMPLING_FACTOR  1
#define FRAME_RETENTION     8

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
//#define HIST_STATIC_PANIC_THRESHOLD  20000 // this many readings in the near range will cause instant panic
//#define HIST_DYNAMIC_PANIC_THRESHOLD  1000 // this many readings suddenly transitioning out of the near range will cause panic
// object avoidance defines
// values for half frame 640x480
#define OBJECT_AVOIDANCE_ERROR_THRESHOLD 80000 // this many readings in the error range (~0-500 mm) will cause instant panic.
#define OBJECT_AVOIDANCE_PANIC_THRESHOLD  4000 // this many readings suddenly transitioning from range1 to range2 will cause panic.
#define OBJECT_AVOIDANCE_RANGE1_START   500 // mm
#define OBJECT_AVOIDANCE_RANGE1_END     699 // mm
#define OBJECT_AVOIDANCE_RANGE2_START   700 // mm
#define OBJECT_AVOIDANCE_RANGE2_END     899 // mm

// pathing defines
#define PATHING_MARKER_STOPPING_DISTANCE 800 // mm. the distance to stop away from markers.

// target recognition defines
#define TARGET_RECOGNITION_MARKER_WIDTH                 50 // mm
#define TARGET_RECOGNITION_MARKER_HEIGHT                50 // mm
#define TARGET_RECOGNITION_MARKER_EPSILON               20 // mm
#define TARGET_RECOGNITION_HAAR_SCALING                1.2 // default 1.1
#define TARGET_RECOGNITION_HAAR_NEIGHBOURS               1 // default 2
#define TARGET_RECOGNITION_EXTRACTION_BINARY_THRESHOLD  64 // grayscale cutoff for binarisation used in marker extraction.
#define TARGET_RECOGNITION_EXTRACTION_SIZE_THRESHOLD    80 // pixel blobs with an area smaller than this are removed.
#define TARGET_RECOGNITION_CASCADE_PATH                "/home/jon/individual_project/haarcascade_marker2_16.xml"

// occupancy analysis defines
#define OCCUPANCY_ANALYSIS_X_START       64 // the number of pixels cropped from the left   of the image before OA starts. 10% of the width.
#define OCCUPANCY_ANALYSIS_X_END        576 // the number of pixels cropped from the right  of the image before OA starts. 10% of the width.
#define OCCUPANCY_ANALYSIS_Y_START      120 // the number of pixels cropped from the top    of the image before OA starts. 25% of the height.
#define OCCUPANCY_ANALYSIS_Y_END        360 // the number of pixels cropped from the bottom of the image before OA starts. 25% of the height.
#define OCCUPANCY_ANALYSIS_RANGE_START  800 // mm. the start of the range to look for legs in.
#define OCCUPANCY_ANALYSIS_RANGE_END   1600 // mm. the end   of the range to look for legs in.




/*--------------------  MACROS  --------------------*/
// None.


/*---------------- CLASS DEFINITION ----------------*/
using namespace xn; // TODO GET RID OF THIS LINE!


class Vision
{
public:
    Vision (void);
    ~Vision (void);
    
    void captureFrame        (void);
    void compressColorFrame  (void);
    void compressDepthFrame  (void);
    void compressFrame       (cv::Mat* frame);
    void compressFrameToDisk (cv::Mat* frame, const char* filename);
    /*void  buildDepthHistogram (void);
    uint32_t queryDepthHistogram (uint16_t v);*/
    sint8_t checkForObstacles (void);
    float getFPS (void);
    
    std::vector<uint8_t> mStreamBuffer;
    

private:
    void loadCameraConfiguration   (void);
    void initialiseCamera          (void);
    void shutdownCamera            (void);
    void setupServers              (void);
    void setupServer               (void);
    void checkForClientConnections (void);
    
    void   createDepthImage (cv::Mat* dst, const uint16_t* src);
//    void   createColorImage (cv::Mat* dst, const uint8_t*  src);
    XnBool fileExists       (const char *fn);
    
    // Vision Variables
    // Frame container(s)
    FrameBuffer     mFrameBuffer;
    // OpenNI image containers
    Context         mContext;
    ScriptNode      mScriptNode;
    
#ifdef DEPTH_STREAM
    DepthGenerator  mDepthGenerator;
    DepthMetaData   mDepthMetaData;
    const uint16_t* mDepthData;
    cv::Mat         mDepthFrame;
    #ifdef DEPTH_STREAMING_ENABLED
    cv::Mat         mDepthStreamingFrame;
    #endif
#endif
#ifdef COLOR_STREAM
    ImageGenerator  mColorGenerator;
    ImageMetaData   mColorMetaData;
    const uint8_t*  mColorData;
    cv::Mat         mColorFrame;
#endif
    XnFPSData       mXnFPS;
    // OpenCV image containers
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
