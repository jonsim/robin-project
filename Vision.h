/**
 * @file        Vision.h
 * @brief       Functions to manage and control the vision (from the Kinect).
 */
#ifndef VISION_H
#define VISION_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
// standard
#include <math.h>
#include <vector>
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
#include <cxcore.h>
#include <highgui.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
// my stuff
#include "FrameBuffer.h"
#include "Histogram.h"




/*-------------------- DEFINES  --------------------*/
#define VISION_XML_CONFIG_PATH       "CameraConfiguration.xml"
#define VISION_XML_CONFIG_PATH_LOCAL "/home/jon/individual_project/CameraConfiguration.xml"
#define COLOR_STREAM                // whether the color video is collected from the Kinect.
#define COLOR_STREAMING_ENABLED     // whether the color video is streamed to connected clients.
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
#define OBJECT_AVOIDANCE_ERROR_THRESHOLD 90000 // this many readings in the error range (~0-500 mm) will cause instant panic.
#define OBJECT_AVOIDANCE_PANIC_THRESHOLD  3000 // this many readings suddenly transitioning from range1 to range2 will cause panic.
#define OBJECT_AVOIDANCE_RANGE1_START   500 // mm
#define OBJECT_AVOIDANCE_RANGE1_END     699 // mm
#define OBJECT_AVOIDANCE_RANGE2_START   700 // mm
#define OBJECT_AVOIDANCE_RANGE2_END     899 // mm

// target recognition defines
#define TARGET_RECOGNITION_RUN_FREQUENCY                10 // every x frames the target recognition module is run.
#define TARGET_RECOGNITION_MARKER_WIDTH                 50 // mm
#define TARGET_RECOGNITION_MARKER_HEIGHT                50 // mm
#define TARGET_RECOGNITION_MARKER_EPSILON               25 // mm
#define TARGET_RECOGNITION_HAAR_SCALING                1.2 // default 1.1
#define TARGET_RECOGNITION_HAAR_NEIGHBOURS               1 // default 2
#define TARGET_RECOGNITION_EXTRACTION_BINARY_THRESHOLD  80 // grayscale cutoff for binarisation used in marker extraction.
#define TARGET_RECOGNITION_EXTRACTION_SIZE_THRESHOLD    80 // pixel blobs with an area smaller than this are removed.
#define TARGET_RECOGNITION_CASCADE_PATH                "/home/jon/individual_project/haarcascade_marker2_16.xml"

// occupancy analysis defines
#define OCCUPANCY_ANALYSIS_X_START        64 // the number of pixels cropped from the left   of the image before OA starts. 10% of the width.
#define OCCUPANCY_ANALYSIS_X_END         576 // the number of pixels cropped from the right  of the image before OA starts. 10% of the width.
#define OCCUPANCY_ANALYSIS_Y_START       120 // the number of pixels cropped from the top    of the image before OA starts. 25% of the height.
#define OCCUPANCY_ANALYSIS_Y_END         360 // the number of pixels cropped from the bottom of the image before OA starts. 25% of the height.
#define OCCUPANCY_ANALYSIS_RANGE_START   800 // mm. the start of the range to look for legs in.
#define OCCUPANCY_ANALYSIS_RANGE_END    1600 // mm. the end   of the range to look for legs in.
#define OCCUPANCY_ANALYSIS_THRESHOLD   50000 // pixels. this many pixels in the given range will result in 'occupancy' being detected. Out of 122800.

#define VERBOSE_PRINTOUTS



/*--------------------  MACROS  --------------------*/
// None.


/*---------------- CLASS DEFINITION ----------------*/
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
    sint8_t checkForObstacles (bool dynamic_check);
    bool    checkForMarkers   (MarkerData* marker_data);
    bool    checkForOccupancy (void);
    float    getFPS (void);
    uint32_t getFrameID (void);
    
    std::vector<uint8_t> mStreamBuffer;
    

private:
    // camera config stuff
    void loadCameraConfiguration   (void);
    void initialiseCamera          (void);
    void shutdownCamera            (void);
    
    // streaming functions
    void   createDepthImage (cv::Mat* dst, const uint16_t* src);
//    void   createColorImage (cv::Mat* dst, const uint8_t*  src);
    
    // marker detection stuff
    void kind_of_subsample (cv::Mat* dst, cv::Mat* src, int x_step, int y_step, int y_start, int y_end);
    void loadMarkerCascade (void);
    float cv_euclidean_distance2 (cv::Point_<int> p1, cv::Point_<int> p2);
    int connectedComponentLabelling (cv::Mat& src, cv::Mat& dst);
    uint32_t* makey_the_hist (cv::Mat& frame);  // TODO use my histogram class
    int suppressNoise (cv::Mat& frame);
    void extractCorners (cv::Mat& frame, cv::Point_<int>* p);
    float getAverageDepthOfRegion (cv::Mat& depthFrame, int* rough_orientation, cv::Point_<int> tl, cv::Point_<int> br);
    void projectPoints (cv::Mat& depthFrame, cv::Point_<int>* p2D, cv::Point3_<float>* p3D, int count);
    bool detectMarkerRegions (cv::Mat& grayscale, std::vector<cv::Rect>* regions);
    bool extractMarkerFromRegions (cv::Mat& color, cv::Mat& grayscale, cv::Mat& depth, std::vector<cv::Rect>* regions, MarkerData* marker_data);    // TODO this only takes color because we draw on it.
    
    // utility functions
    XnBool fileExists       (const char *fn);
    
    // Variables
    // Frame Buffer
    FrameBuffer     mFrameBuffer;
    // Marker detection stuff.
    cv::CascadeClassifier mMarkerCascade;
    std::vector<cv::Rect> mMarkerRegions;
    // OpenNI data capture containers (for various things).
    xn::Context     mContext;
    xn::ScriptNode  mScriptNode;
#ifdef DEPTH_STREAM
    xn::DepthGenerator mDepthGenerator;
    xn::DepthMetaData  mDepthMetaData;
    const uint16_t*    mDepthData;
    cv::Mat            mDepthFrame;
    #ifdef DEPTH_STREAMING_ENABLED
    cv::Mat            mDepthStreamingFrame;
    #endif
#endif
#ifdef COLOR_STREAM
    xn::ImageGenerator mColorGenerator;
    xn::ImageMetaData  mColorMetaData;
    const uint8_t*     mColorData;
    cv::Mat            mColorFrame;
    cv::Mat            mGrayscaleFrame;
#endif
    XnFPSData   mXnFPS;
};

#endif
