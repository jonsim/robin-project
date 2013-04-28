#include "Vision.h"


/// @brief  Constructor.
Vision::Vision (void) : mFrameBuffer(IMAGE_WIDTH, IMAGE_HEIGHT, SUBSAMPLING_FACTOR, FRAME_RETENTION),
#ifdef DEPTH_STREAMING_ENABLED
    #ifdef DEPTH_COLORED
                        mDepthStreamingFrame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3),
    #else
                        mDepthStreamingFrame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1),
    #endif
#endif
#ifdef COLOR_STREAM
                        mColorFrame(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3)
#endif
{
    loadCameraConfiguration();
    initialiseCamera();
}


/// @brief  Deconstructor.
Vision::~Vision (void)
{
    shutdownCamera();
}


/// @brief  Locates the camera's configuration file and, if it exists, loads it.
///         THIS FUNCTION DOES NOT INITIALISE THE CAMERA: see initialiseCamera().
void Vision::loadCameraConfiguration (void)
{
    EnumerationErrors errors;
    XnStatus          retVal = XN_STATUS_OK;

    // Locate config file.
    printf("Reading config file... ");
    fflush(stdout);
    const char *fn = NULL;
    if (fileExists(VISION_XML_CONFIG_PATH))
        fn = VISION_XML_CONFIG_PATH;
    else if (fileExists(VISION_XML_CONFIG_PATH_LOCAL))
        fn = VISION_XML_CONFIG_PATH_LOCAL;
    else
    {
        printf("aborting.\n  ERROR: Could not find '%s' nor '%s'.\n" , VISION_XML_CONFIG_PATH, VISION_XML_CONFIG_PATH_LOCAL);
        return;
    }
    
    // Load config file.
    retVal = mContext.InitFromXmlFile(fn, mScriptNode, &errors);
    if (retVal == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("aborting.\n  ERROR: %s\n", strError);
        exit(EXIT_FAILURE);
    }
    CHECK_RETURN_XN(retVal, "Open");

    printf("done.\n");
}


/// @brief  Uses a loaded configuration (from mContext) to initialise and configure the camera.
///         A CONFIGURATION MUST BE LOADED FIRST: see loadCameraConfiguration().
void Vision::initialiseCamera (void)
{
    XnStatus retVal = XN_STATUS_OK;
    
    // initialise and validate main variables
    printf("Initialising camera components... ");
    fflush(stdout);
    
#ifdef DEPTH_STREAM
    retVal = mContext.FindExistingNode(XN_NODE_TYPE_DEPTH, mDepthGenerator);
    CHECK_RETURN_XN(retVal, "Find depth generator");
#endif
#ifdef COLOR_STREAM
    retVal = mContext.FindExistingNode(XN_NODE_TYPE_IMAGE, mColorGenerator);
    CHECK_RETURN_XN(retVal, "Find color generator");
#endif
    retVal = xnFPSInit(&mXnFPS, 180);
    CHECK_RETURN_XN(retVal, "FPS Init");
    
    printf("done.\n");
}


/// @brief  Shuts down the camera, releasing all objects associated with it.
void Vision::shutdownCamera (void)
{
#ifdef DEPTH_STREAM
    mDepthGenerator.Release();
#endif
#ifdef COLOR_STREAM
    mColorGenerator.Release();
#endif
    mScriptNode.Release();
    mContext.Release();
}


/// @brief  Waits until new data is available and then captures a single frame of depth data from
///         the camera and updates the mDepthData object variable with the new data.
void Vision::captureFrame (void)
{
    static XnStatus retVal = XN_STATUS_OK;
    
    // Update the data.
    retVal = mContext.WaitAndUpdateAll();
    if (retVal != XN_STATUS_OK)
    {
        printf("UpdateData failed (%s). Continuing anyway.\n", xnGetStatusString(retVal));
        return;
    }

    // Read the data into our containers.
#ifdef DEPTH_STREAM
    mDepthGenerator.GetMetaData(mDepthMetaData);
    mDepthData = (const uint16_t*) mDepthMetaData.Data();
    mDepthFrame = cv::Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1, (uint16_t*) mDepthData);
#endif
#ifdef COLOR_STREAM
    mColorGenerator.GetMetaData(mColorMetaData);
    mColorData = (const uint8_t*) mColorMetaData.Data();
    memcpy(mColorFrame.data, mColorData, IMAGE_HEIGHT * IMAGE_WIDTH * 3);
    cv::cvtColor(mColorFrame, mColorFrame, CV_RGB2BGR);
#endif
    
    // Pop this data into the frame buffer.
    mFrameBuffer.insert(mDepthData);
}


void Vision::compressDepthFrame (void)
{
#if defined(DEPTH_STREAM) && defined(DEPTH_STREAMING_ENABLED)
    createDepthImage(&mDepthStreamingFrame, mDepthData);
    compressFrame(&mDepthStreamingFrame);
#else
    printf("Tried to compress depth stream when no stream was present.\n");
#endif
}


void Vision::compressColorFrame (void)
{
#if defined(COLOR_STREAM) && defined(COLOR_STREAMING_ENABLED)
//    createColorImage(&mColorImage, mColorData);
    compressFrame(&mColorFrame);
#else
    printf("Tried to compress color stream when no stream was present.\n");
#endif
}


/// @brief  Compresses the currently saved depth frame (loaded from the captureFrame() method) to a
///         JPEG which resides in the mStreamingDataJPEG object variable.
void Vision::compressFrame (cv::Mat* frame)
{
    static int paramsArray[] = {CV_IMWRITE_JPEG_QUALITY, COMPRESSION_QUALITY};
    static std::vector<int> paramsVector(paramsArray, paramsArray + sizeof(paramsArray) / sizeof(int));
    
    // Compress it to a JPEG.
    cv::imencode(".JPEG", *frame, mStreamBuffer, paramsVector);
}


/// @brief  Compresses the currently saved depth frame (loaded from the captureFrame() method) to a
///         JPEG which resides in the mStreamingDataJPEG object variable.
/// @param  filename    The name with which to save the frame. The extension may be jpg, png, ppm, pgm or pbm.
void Vision::compressFrameToDisk (cv::Mat* frame, const char* filename)
{
    static int paramsArray[] = {CV_IMWRITE_JPEG_QUALITY, COMPRESSION_QUALITY};
    static std::vector<int> paramsVector(paramsArray, paramsArray + sizeof(paramsArray) / sizeof(int));
    
    // Produce a human-viewable colour representation of the depth data
    //createColourDepthImage(&mStreamingDepthRaw, mDepthData);
    
    // Compress it to a JPEG
    int fnl = strlen(filename);
    if (filename[fnl-3]=='j' && filename[fnl-2]=='p' && filename[fnl-1]=='g')
        cv::imwrite(filename, *frame, paramsVector);
    else
        cv::imwrite(filename, *frame);
}


/// @brief  Records this function call as a frame and returns the current FPS. It is therefore vital
///         this function is called EXACTLY once per frame, no more, no less.
/// @return The current FPS.
float Vision::getFPS (void)
{
    xnFPSMarkFrame(&mXnFPS);
    return xnFPSCalc(&mXnFPS);
}


/// @brief  PANIC YET?!?!??!?!???
/// @return a value representing whether or not we should be about to freak the shit out. a return
///         of 0 suggests we're all cool, a return of 1 means there is legit grounds for panic on
///         the right hand side and a return of -1 means we need to worry about something on our left.
///         in the case that there is something scary on both sides a value of 2 will be returned.
sint8_t Vision::checkForObstacles (void)
{
    Histogram* newLHistogram;
    Histogram* newRHistogram;
    Histogram* oldLHistogram;
    Histogram* oldRHistogram;
    uint32_t   newLHistogramRange1, newLHistogramRange2, newLHistogramError;
    uint32_t   newRHistogramRange1, newRHistogramRange2, newRHistogramError;
    uint32_t   oldLHistogramRange1, oldLHistogramRange2;
    uint32_t   oldRHistogramRange1, oldRHistogramRange2;
    sint8_t    obstacleDetected = 0;
    
    // fill the histogram pointers appropriately.
    mFrameBuffer.retrieveHistograms(0,                 &newLHistogram, &newRHistogram);
    mFrameBuffer.retrieveHistograms(FRAME_RETENTION-1, &oldLHistogram, &oldRHistogram);
    
    // check we're all properly initialised
    if (newLHistogram == NULL || newRHistogram == NULL || oldLHistogram == NULL || oldRHistogram == NULL)
        return 0;
    
    // retrieve the ranges
    newLHistogramError  = newLHistogram->get(0);
    newRHistogramError  = newRHistogram->get(0);
    newLHistogramRange1 = newLHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
    newLHistogramRange2 = newLHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
    newRHistogramRange1 = newRHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
    newRHistogramRange2 = newRHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
    oldLHistogramRange1 = oldLHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
    oldLHistogramRange2 = oldLHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
    oldRHistogramRange1 = oldRHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
    oldRHistogramRange2 = oldRHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
    
    // STATIC THREAT DETECTION
    if (newLHistogramError > OBJECT_AVOIDANCE_ERROR_THRESHOLD)   // left
        obstacleDetected = -1;
    if (newRHistogramError > OBJECT_AVOIDANCE_ERROR_THRESHOLD)   // right
        obstacleDetected = (obstacleDetected == -1) ? 2 :  1;
    
    // DYNAMIC THREAT DETECTION
/*    printf("nL: %d,%d. oL: %d,%d. nR: %d,%d. oR: %d,%d.\n", newLHistogramRange1, newLHistogramRange2,
                                                            oldLHistogramRange1, oldLHistogramRange2, 
                                                            newRHistogramRange1, newRHistogramRange2,
                                                            oldRHistogramRange1, oldRHistogramRange2 );*/
    if ((newLHistogramRange2 < (oldLHistogramRange2 - OBJECT_AVOIDANCE_PANIC_THRESHOLD)) &&     // left
        (newLHistogramRange1 > (oldLHistogramRange1 + OBJECT_AVOIDANCE_PANIC_THRESHOLD))    )
        obstacleDetected = (obstacleDetected ==  1) ? 2 : -1;
    if ((newRHistogramRange2 < (oldRHistogramRange2 - OBJECT_AVOIDANCE_PANIC_THRESHOLD)) &&     // right
        (newRHistogramRange1 > (oldRHistogramRange1 + OBJECT_AVOIDANCE_PANIC_THRESHOLD))    )
        obstacleDetected = (obstacleDetected == -1) ? 2 :  1;
    
    return obstacleDetected;
}

/*void Vision::buildDepthHistogram (void)
{
/    uint16_t x, y;
    uint32_t one_step_offset, i;
    
    // zero the histogram
    for (i = 0; i < MAX_DEPTH; i++)
        mDepthHistogram[i] = 0;
    
    // count the values
    for (y = 0, one_step_offset = 0; y < IMAGE_HEIGHT; y++)
        for (x = 0; x < IMAGE_WIDTH; x++, one_step_offset++)
            mDepthHistogram[mDepthData[one_step_offset]]++;/
}

uint32_t Vision::queryDepthHistogram (uint16_t v)
{
//    return (v < MAX_DEPTH) ? mDepthHistogram[v] : 0;
    return mHistogram.get(v);
}*/



/// @brief  Creates a colour image (as a cv::Mat) representation of the depth data in the given array.
/// @param  dst   The cv::Mat into which the colour representation will be saved. The size of this
///               is defined by the IMAGE_HEIGHT and IMAGE_WIDTH values. The cv::Mat needs to be
///               in the CV_8UC3 mode (8 bit RGB).
/// @param  src   A pointer to the uint16_t array of depth data. The size of this is defined by the
///               IMAGE_HEIGHT and IMAGE_WIDTH values.
void Vision::createDepthImage (cv::Mat* dst, const uint16_t* src)
{
#ifdef DEPTH_COLORED
    static const float scaling_factor = 4800/1530;
    uint16_t x=0, y=0;
    uint32_t one_step_offset=0, three_step_offset=0;
    uint16_t v;
    uint8_t  r, g, b;
    uint8_t* loc;
    
    for (y=0, one_step_offset=0, three_step_offset=0; y < IMAGE_HEIGHT; y++)
    {
        for (x = 0; x < IMAGE_WIDTH; x++, one_step_offset++, three_step_offset+=3)
        {
            v = (uint16_t) src[one_step_offset];
            
            if (v == 0)
            {
                r = 0;
                g = 0;
                b = 0;
            }
            else
            {
                v -= 400;
                v = (v < 4800) ? v / scaling_factor : 1530;
                // H' takes values between 0-1530
                // H' =    0- 255  RGB=   255, 0-255, 0
                // H' =  255- 510  RGB= 255-0,   255, 0
                // H' =  510- 765  RGB=     0,   255, 0-255
                // H' =  765-1020  RGB=     0, 255-0, 255
                // H' = 1020-1275  RGB= 0-255,     0, 255
                // H' = 1275-1530  RGB=   255,     0, 255-0
                
                if (v < 255)
                {
                    r = 255u;
                    g = (uint8_t) v;  // g increases to 255
                    b =   0u;
                }
                else if (v < 510)
                {
                    r = (uint8_t) (510 - v);  // r falls to 0
                    g = 255u;
                    b =   0u;
                }
                else if (v < 765)
                {
                    r =   0u;
                    g = 255u;
                    b = (uint8_t) (v - 510);  // b increases to 255
                }
                else if (v < 1020)
                {
                    r =   0u;
                    g = (uint8_t) (1020 - v);  // g falls to 0
                    b = 255u;
                }
                else if (v < 1275)
                {
                    r = (uint8_t) (v - 1020);  // r increases to 255
                    g =   0u;
                    b = 255u;
                }
                else  // v <= 1530
                {
                    r = 255u;
                    g =   0u;
                    b = (uint8_t) (1530 - v);  // b falls to 0
                }
            }
            
            loc = dst->data + three_step_offset;
            loc[0] = b;
            loc[1] = g;
            loc[2] = r;
        }
    }
#else
    static const float scaling_factor = 10000/255;
    uint16_t x=0, y=0;
    uint32_t one_step_offset=0;
    
    for (y=0, one_step_offset=0; y < IMAGE_HEIGHT; y++)
    {
        for (x = 0; x < IMAGE_WIDTH; x++, one_step_offset++)
        {
            dst->data[one_step_offset] = (uint16_t) (src[one_step_offset] * scaling_factor);
        }
    }
#endif
}


/// @brief  TBD
/*void Vision::createColorImage (cv::Mat* dst, const uint8_t* src)
{
    memcpy(dst->data, src, IMAGE_HEIGHT * IMAGE_WIDTH * 3);
    cv::cvtColor(*dst, *dst, CV_RGB2BGR);
}*/


/// @brief  Checks whether a given file exists on the system.
/// @param  fn  The file path to check.
/// @return Whether the file exists.
XnBool Vision::fileExists(const char *fn)
{
    XnBool exists;
    xnOSDoesFileExist(fn, &exists);
    return exists;
}




/// @brief  Unit testing.
/*int main (void)
{
    Vision v;
    
    while (!xnOSWasKeyboardHit())
    {
        v.captureFrame();
        v.compressFrame();
    }
}*/
