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
    loadMarkerCascade();
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
    xn::EnumerationErrors errors;
    XnStatus retVal = XN_STATUS_OK;

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
    
    // print some stuffs
    double pixel_size;
    XnUInt64 focal_length;
    mDepthGenerator.GetRealProperty("ZPPS", pixel_size);
    mDepthGenerator.GetIntProperty( "ZPD",  focal_length);
    printf("ZPPS=%f, ZPD=%d\n", pixel_size, (int) focal_length);
    
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


/// @brief  Fetches the ID of the current frame, as recorded by the depth camera. As this is updated
///         at the same time as the colour camera this can be considered the system-wide frame ID.
/// @return The current frame ID.
uint32_t Vision::getFrameID (void)
{
    return (uint32_t) mDepthMetaData.FrameID();
}


/// @brief  PANIC YET?!?!??!?!???
/// @return a value representing whether or not we should be about to freak the shit out. a return
///         of 0 suggests we're all cool, a return of 1 means there is legit grounds for panic on
///         the right hand side and a return of -1 means we need to worry about something on our left.
///         in the case that there is something scary on both sides a value of 2 will be returned.
sint8_t Vision::checkForObstacles (bool dynamic_check)
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
//    printf("  retrieving histograms\n");
    mFrameBuffer.retrieveHistograms(0,                 &newLHistogram, &newRHistogram);
    mFrameBuffer.retrieveHistograms(FRAME_RETENTION-2, &oldLHistogram, &oldRHistogram);
//    printf("  - retrieved nL (%x), nR (%x), oL (%x), oR (%x)\n", (uint32_t) newLHistogram, (uint32_t) newRHistogram, (uint32_t) oldLHistogram, (uint32_t) oldRHistogram);
    
    // check we're all properly initialised
    if (newLHistogram == NULL || newRHistogram == NULL)
        return 0;
    else if (oldLHistogram == NULL || oldRHistogram == NULL)
        dynamic_check = false;
    
    // retrieve the ranges
//    printf("  getting the ranges\n");
    newLHistogramError  = newLHistogram->get(0);
    newRHistogramError  = newRHistogram->get(0);
    
    // STATIC THREAT DETECTION
//    printf("  static threat detection\n");
    if (newLHistogramError > OBJECT_AVOIDANCE_ERROR_THRESHOLD)   // left
        obstacleDetected = -1;
    if (newRHistogramError > OBJECT_AVOIDANCE_ERROR_THRESHOLD)   // right
        obstacleDetected = (obstacleDetected == -1) ? 2 :  1;
#ifdef VERBOSE_PRINTOUTS
    if (obstacleDetected)
        printf("static error (%d, %d)\n", newLHistogramError, newRHistogramError);
#endif
    
    // DYNAMIC THREAT DETECTION
//    printf("  dynamic threat detection\n");
/*    printf("nL: %d,%d. oL: %d,%d. nR: %d,%d. oR: %d,%d.\n", newLHistogramRange1, newLHistogramRange2,
                                                            oldLHistogramRange1, oldLHistogramRange2, 
                                                            newRHistogramRange1, newRHistogramRange2,
                                                            oldRHistogramRange1, oldRHistogramRange2 );*/
    if (dynamic_check)
    {
        newLHistogramRange1 = newLHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
        newLHistogramRange2 = newLHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
        newRHistogramRange1 = newRHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
        newRHistogramRange2 = newRHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
        
        oldLHistogramRange1 = oldLHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
        oldLHistogramRange2 = oldLHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
        oldRHistogramRange1 = oldRHistogram->getRange(OBJECT_AVOIDANCE_RANGE1_START, OBJECT_AVOIDANCE_RANGE1_END);
        oldRHistogramRange2 = oldRHistogram->getRange(OBJECT_AVOIDANCE_RANGE2_START, OBJECT_AVOIDANCE_RANGE2_END);
        
        if ((newLHistogramRange2 < (oldLHistogramRange2 - OBJECT_AVOIDANCE_PANIC_THRESHOLD)) &&     // left
            (newLHistogramRange1 > (oldLHistogramRange1 + OBJECT_AVOIDANCE_PANIC_THRESHOLD))    )
        {
            obstacleDetected = (obstacleDetected ==  1) ? 2 : -1;
#ifdef VERBOSE_PRINTOUTS
            printf("dynamic left (%d, %d)\n", oldLHistogramRange2 - newLHistogramRange2, newLHistogramRange1 - oldLHistogramRange1);
#endif
        }
        if ((newRHistogramRange2 < (oldRHistogramRange2 - OBJECT_AVOIDANCE_PANIC_THRESHOLD)) &&     // right
            (newRHistogramRange1 > (oldRHistogramRange1 + OBJECT_AVOIDANCE_PANIC_THRESHOLD))    )
        {
            obstacleDetected = (obstacleDetected == -1) ? 2 :  1;
#ifdef VERBOSE_PRINTOUTS
            printf("dynamic right (%d, %d)\n",  oldRHistogramRange2 - newRHistogramRange2, newRHistogramRange1 - oldRHistogramRange1);
#endif
        }
    }
    
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


/// @brief  TODO
void Vision::loadMarkerCascade (void)
{
    int retVal;
    retVal = mMarkerCascade.load(TARGET_RECOGNITION_CASCADE_PATH);
    
    if (retVal == 0)
    {
        perror("MarkerCascade load");
        exit(EXIT_FAILURE);
    }
}


/// @brief  TODO
bool Vision::checkForMarkers (MarkerData* marker_data)
{
    //bool marker_seen, marker_found = false;
    
    // First make a grayscale map of the image. Equalising the histogram has the effect of removing
    // a lot of the effects of differing light levels and increasing contrast.
    cv::cvtColor(mColorFrame, mGrayscaleFrame, CV_BGR2GRAY);
    cv::equalizeHist(mGrayscaleFrame, mGrayscaleFrame);
    
    // detect markers
    detectMarkerRegions(mGrayscaleFrame, &mMarkerRegions);
    if (mMarkerRegions.size() > 0)
    {
//        printf("Some markers were found, extracting...\n");
        return extractMarkerFromRegions(mColorFrame, mGrayscaleFrame, mDepthFrame, &mMarkerRegions, marker_data);
    }
    return false;
}


/// @brief  TODO
bool Vision::detectMarkerRegions (cv::Mat& grayscale, std::vector<cv::Rect>* regions)
{
    int i;
    std::vector<cv::Rect> internal_regions;
//    cv::Mat gs_scaled(240, 320, CV_8UC1);
//    kind_of_subsample(&gs_scaled, &grayscale, 2, 2, 0, 480);

    // Run the haar cascade to detect the marker.
    mMarkerCascade.detectMultiScale(grayscale, internal_regions, TARGET_RECOGNITION_HAAR_SCALING, TARGET_RECOGNITION_HAAR_NEIGHBOURS, CV_HAAR_SCALE_IMAGE);

    // If we haven't found any markers (most of the time), return.
    if (internal_regions.size() == 0)
        return false;

    // Replace the supplied list with the found marker(s).
    regions->clear();
    for (i = 0; i < (int) internal_regions.size(); i++)
    {
        regions->push_back(internal_regions[i]);
        /*(*regions)[i].x *= 2;
        (*regions)[i].y *= 2;
        (*regions)[i].width *= 2;
        (*regions)[i].height *= 2;*/
    }
    return true;
}


/// @brief  TODO
bool Vision::extractMarkerFromRegions (cv::Mat& color, cv::Mat& grayscale, cv::Mat& depth, std::vector<cv::Rect>* regions, MarkerData* marker_data)
{
    int i, j;
//    int k;
    cv::Mat clipped_marker;
    cv::Point_<int>    marker_corners2[4];
//    cv::Point_<int>    marker_corners2_temp[4];
    cv::Point3_<float> marker_corners3[4];
//    cv::Point3_<float> marker_corners3_temp[4];
    int rough_orientation;
    float marker_width3d, marker_height3d, marker_depth3d;
    float marker_orientation3d, marker_unfoldedwidth, marker_unfoldedheight;
    cv::Point3_<float> marker_position3d;
    int regions_size = regions->size();

    // Okay, so we detected something - go through all the detected objects and analyse their likelihood of being our marker.
    for (i = 0; i < regions_size; i++)
    {
        // binarise the image around the given threshold.
        cv::threshold(grayscale((*regions)[i]), clipped_marker, TARGET_RECOGNITION_EXTRACTION_BINARY_THRESHOLD, 255, cv::THRESH_BINARY_INV);

        // 'cluster' the image into connected regions. remove those regions that are too small (and are thus probably noise).
        cv::Mat region_marker(clipped_marker.size(), CV_8UC1);
        //int region_count = connectedComponentLabelling(clipped_marker, region_marker);
        //int region_count_reduction = suppressNoise(region_marker);
        int region_count = connectedComponentLabelling(clipped_marker, region_marker);
        int region_count_reduction = suppressNoise(region_marker);
        printf("detected %d regions, reduced to %d\n", region_count, region_count - region_count_reduction);


        // threshold this result. the regions will be numbers > 1, we don't care about what regions there are anymore, only
        // that there is a foreground region (the marker, minus noise) and a background region (not the marker).
        cv::threshold(region_marker, region_marker, 0, 255, cv::THRESH_BINARY);

        cv::imwrite("shit.png", region_marker);
        
        // from this thresholded image, extract the 4 corners that make up the marker (if this is the marker at all).
        extractCorners(region_marker, marker_corners2);
        // Convert these to global image co-ordinates.
        for (j = 0; j < 4; j++)
            marker_corners2[j] += (*regions)[i].tl();
        
        // get the minimum box
        float average_depth = getAverageDepthOfRegion(depth, &rough_orientation, cv::Point_<int>(MAX(marker_corners2[0].x, marker_corners2[1].x), MAX(marker_corners2[0].y, marker_corners2[2].y)),
                                                                                 cv::Point_<int>(MIN(marker_corners2[2].x, marker_corners2[3].x), MIN(marker_corners2[1].y, marker_corners2[3].y)) );
        for (j = 0; j < 4; j++)
        {
            marker_corners3[j].x = marker_corners2[j].x * average_depth * ((PIXEL_SIZE * 2) / FOCAL_LENGTH);
            marker_corners3[j].y = marker_corners2[j].y * average_depth * ((PIXEL_SIZE * 2) / FOCAL_LENGTH);
            marker_corners3[j].z = average_depth;
        }
        /*
        // convert these 4 2D points to 4 3D points.
        for (j = 0; j < 4; j++)
        {
            int y_sign = (j == 0 || j == 2) ? -1 : 1;
            int x_sign = (j > 1)            ? -1 : 1;
            
            marker_corners2_temp[0] = marker_corners2[j];
            marker_corners2_temp[1] = marker_corners2[j] + cv::Point_<int>(x_sign,      0);
            marker_corners2_temp[2] = marker_corners2[j] + cv::Point_<int>(0,      y_sign);
            marker_corners2_temp[3] = marker_corners2[j] + cv::Point_<int>(x_sign, y_sign);
            projectPoints(depth, marker_corners2_temp, marker_corners3_temp, 4);
            
            int non_zero_count = 0;
            for (k = 0; k < 4; k++)
                if (marker_corners3_temp[k].z != 0)
                    non_zero_count++;
            if (non_zero_count == 0)
            {
                marker_corners3[j].x = 0;
                marker_corners3[j].y = 0;
                marker_corners3[j].z = 0;
            }
            else
            {
                marker_corners3[j].x = (marker_corners3_temp[0].x + marker_corners3_temp[1].x + marker_corners3_temp[2].x + marker_corners3_temp[3].x) / (float) non_zero_count;
                marker_corners3[j].y = (marker_corners3_temp[0].y + marker_corners3_temp[1].y + marker_corners3_temp[2].y + marker_corners3_temp[3].y) / (float) non_zero_count;
                marker_corners3[j].z = (marker_corners3_temp[0].z + marker_corners3_temp[1].z + marker_corners3_temp[2].z + marker_corners3_temp[3].z) / (float) non_zero_count;
            }
        }
        
        //projectPoints(depth, marker_corners2, marker_corners3, 4);
        if (marker_corners3[0].z == 0)
        {
            marker_corners3[0].z = marker_corners3[1].z;
            marker_corners3[0].x = marker_corners3[1].x;
            marker_corners3[0].y = marker_corners3[2].y;
        }
        if (marker_corners3[1].z == 0)
        {
            marker_corners3[1].z = marker_corners3[0].z;
            marker_corners3[1].x = marker_corners3[0].x;
            marker_corners3[1].y = marker_corners3[3].y;
        }
        if (marker_corners3[2].z == 0)
        {
            marker_corners3[2].z = marker_corners3[3].z;
            marker_corners3[2].x = marker_corners3[3].x;
            marker_corners3[2].y = marker_corners3[0].y;
        }
        if (marker_corners3[3].z == 0)
        {
            marker_corners3[3].z = marker_corners3[3].z;
            marker_corners3[3].x = marker_corners3[3].x;
            marker_corners3[3].y = marker_corners3[1].y;
        }
        */

        // extract their 3D info.
        marker_width3d  = ((marker_corners3[2].x - marker_corners3[0].x) + (marker_corners3[3].x - marker_corners3[1].x)) / 2.0f;
        marker_height3d = ((marker_corners3[0].y - marker_corners3[1].y) + (marker_corners3[2].y - marker_corners3[3].y)) / 2.0f;
//        marker_depth3d  = ((marker_corners3[0].z - marker_corners3[2].z) + (marker_corners3[1].z - marker_corners3[3].z)) / 2.0f;
        marker_depth3d = (marker_width3d < marker_height3d) ? rough_orientation * sqrt(marker_height3d*marker_height3d - marker_width3d*marker_width3d) : 0.0;
//        printf("marker_depth3d = %.1f (average of %.1f, %.1f, %.1f, %.1f)\n", marker_depth3d, marker_corners3[0].z, marker_corners3[1].z, marker_corners3[2].z, marker_corners3[3].z);
        marker_position3d = cv::Point3_<float>(((marker_corners3[0].x + marker_corners3[1].x + marker_corners3[2].x + marker_corners3[3].x) / 4.0f),
                                               ((marker_corners3[0].y + marker_corners3[1].y + marker_corners3[2].y + marker_corners3[3].y) / 4.0f),
                                               ((marker_corners3[0].z + marker_corners3[1].z + marker_corners3[2].z + marker_corners3[3].z) / 4.0f) );
        marker_orientation3d  = (float) (-atan(marker_depth3d / marker_width3d) * (180.0 / PI));
        marker_unfoldedwidth  = sqrt(marker_width3d*marker_width3d   + marker_depth3d*marker_depth3d);
        marker_unfoldedheight = sqrt(marker_height3d*marker_height3d + marker_depth3d*marker_depth3d);

        // check if the thing we're looking at could actually be a marker
        if ((marker_unfoldedwidth  > (TARGET_RECOGNITION_MARKER_WIDTH  - TARGET_RECOGNITION_MARKER_EPSILON)) &&
            (marker_unfoldedwidth  < (TARGET_RECOGNITION_MARKER_WIDTH  + TARGET_RECOGNITION_MARKER_EPSILON)) &&
            (marker_unfoldedheight > (TARGET_RECOGNITION_MARKER_HEIGHT - TARGET_RECOGNITION_MARKER_EPSILON)) &&
            (marker_unfoldedheight < (TARGET_RECOGNITION_MARKER_HEIGHT + TARGET_RECOGNITION_MARKER_EPSILON))   )
        {
            //printf("  MARKER DETECTED.  Position = (%.1f, %.1f, %.1f). Orientation = %.1f. Unfolded-size (wxh) = (%.1fx%.1f)\n", marker_position3d.x, marker_position3d.y, marker_position3d.z, marker_orientation3d, marker_unfoldedwidth, marker_unfoldedheight);
            
            // it is a marker! OMG! return it! (ignoring any subsequent marker objects... how could we discern them anyway?)
            marker_data->position    = marker_position3d;
            marker_data->orientation = marker_orientation3d;

            // also draw it while we can
            cv::line(color, marker_corners2[0], marker_corners2[1], cv::Scalar(255,0,255), 2, 8, 0);
            cv::line(color, marker_corners2[1], marker_corners2[3], cv::Scalar(255,0,255), 2, 8, 0);
            cv::line(color, marker_corners2[3], marker_corners2[2], cv::Scalar(0,255,255), 2, 8, 0);
            cv::line(color, marker_corners2[2], marker_corners2[0], cv::Scalar(0,255,255), 2, 8, 0);

            return true;
        }
        else
        {
//            printf("  MARKER LIKE OBJECT IGNORED 2d: (%.1fx%.1f).\n", marker_unfoldedwidth, marker_unfoldedheight);
            cv::rectangle(color, (*regions)[i], cv::Scalar(0,255,255), 2, 8, 0);
            regions->erase(regions->begin() + i);
            regions_size--;
            i--;
        }
    }
    return false;
}


// TODO: we can output the areas of each region from this function but opencv wants to be a little diva and fuck up the heap when we try...
int Vision::connectedComponentLabelling (cv::Mat& src, cv::Mat& dst)
{
    int x, y, c_index, n_index, w_index, n_label, w_label;
    int xres = src.cols, yres = src.rows;
    int region_count = 0;
    int i;
    uint8_t region_label;
    std::vector< std::pair<int, int> > regions_equivalence;

    //printf("executing first pass\n");
    for (c_index = 0, y = 0; y < yres; y++)
    {
        for (         x = 0; x < xres; x++, c_index++)
        {
            // if the cell has data
            if (src.data[c_index] != 0)
            {
                // neighbours
                n_index = c_index - xres;
                n_label = (y == 0) ? 0 : dst.data[n_index];
                w_index = c_index - 1;
                w_label = (x == 0) ? 0 : dst.data[w_index];
                // if it has a labelled north.
                if (n_label)
                {
                    // and a labelled west.
                    if (w_label)
                    {
                        // and they're different.
                        if (n_label != w_label)
                        {
                            // mark the regions as equivalent (unless they've already been marked).
                            bool regions_already_equivalent = false;
                            for (i = 0; i < (int) regions_equivalence.size(); i++)
                            {
                                regions_already_equivalent |= (regions_equivalence[i].first == n_label && regions_equivalence[i].second == w_label) ||
                                                              (regions_equivalence[i].first == w_label && regions_equivalence[i].second == n_label);
                            }
                            if (!regions_already_equivalent)
                            {
                                regions_equivalence.push_back(std::pair<uint32_t, uint32_t>(MIN(n_label, w_label), MAX(n_label, w_label)));
                            }
                        }
                    }
                    // give it the north label.
                    region_label = (uint8_t) n_label;
                }
                else if (w_label)
                {
                    // otherwise mark it with the west label.
                    region_label = (uint8_t) w_label;
                }
                else
                {
                    // otherwise make it a new region.
                    region_label = (uint8_t) ++region_count;
                }
            }
            else
            {
                // no region for you.
                region_label = 0;
            }
            // save it.
            dst.data[c_index] = region_label;
        }
    }
    //printf("finished first pass\n");
    
    // merge equivalent regions down. we also compress the region labels back to a contiguous range, because
    // equivalent regions will have used extras, GOD DAMN IT MEG.
    if (regions_equivalence.size() > 0)
    {
        uint32_t j;
        uint32_t new_region_count;
        uint32_t duplicate_index;
        std::vector<uint32_t> region_mapping;
        region_mapping.push_back(0);

        // go through all the old regions
        for (i = 1, new_region_count = 1; i <= region_count; i++)
        {
            // check if the region currently being considered is equivalent to any other, lower numbered, regions.
            for (j = 0, duplicate_index = 0; j < regions_equivalence.size(); j++)
            {
                if (regions_equivalence[j].second == i)
                {
                    duplicate_index = regions_equivalence[j].first;
                    break;
                }
            }

            // if it is a duplicate region, set it to the old duplicate and don't update the new_region_count;
            if (duplicate_index != 0)
            {
                if (region_mapping.size() <= duplicate_index)
                    printf("uh oh... region_mapping.size()=%d, duplicate_index=%d\n", region_mapping.size(), duplicate_index);
                region_mapping.push_back(region_mapping[duplicate_index]);
            }
            else
                region_mapping.push_back(new_region_count++);
        }
        // apply the mapping
        for (i = 0; i < src.size().area(); i++)
            dst.data[i] = (uint8_t) region_mapping[dst.data[i]];
        //printf("finished second pass\n");
        return new_region_count;
    }
    else
    {
        //printf("no second pass necessary\n");
        return region_count+1;
    }
}


/// @brief  TODO
uint32_t* Vision::makey_the_hist (cv::Mat& frame)
{
    uint32_t* r = (uint32_t*) calloc(256, sizeof(uint32_t));
    uint32_t lim = frame.size().area();
    for (uint32_t i = 0; i < lim; i++)
        r[frame.data[i]]++;
    return r;
}


/// @brief  TODO
int Vision::suppressNoise (cv::Mat& frame)
{
    uint32_t v;
    uint32_t* hist;
    int reductions = 0;
    uint8_t i;
    int j;

    hist = makey_the_hist(frame);
    for (i = 1; i < 256; i++)
    {
        v = hist[i];
        if (v == 0)
            break;

        if (v < TARGET_RECOGNITION_EXTRACTION_SIZE_THRESHOLD)
        {
            reductions++;
            for (j = 0; j < frame.size().area(); j++)
                if (frame.data[j] == i)
                    frame.data[j] = 0;
        }
    }
    free(hist);

    return reductions;
}


/// @brief  TODO
void Vision::extractCorners (cv::Mat& frame, cv::Point_<int>* p)
{
    int x=0, y=0;
    int xres  = frame.cols, yres  = frame.rows;
    int hxres = xres / 2,   hyres = yres / 2;
    cv::Point_<int> candidate1,       candidate2;
    float           candidate1_dist2, candidate2_dist2;
    // add the images corner points
    cv::Point_<int> frame_tl(0,      0);
    cv::Point_<int> frame_tr(xres-1, 0);
    cv::Point_<int> frame_bl(0,      yres-1);
    cv::Point_<int> frame_br(xres-1, yres-1);


    // find top left corner
    // find leftmost
    for (x = 0; x < hxres; x++)
        for (y = 0; y < hyres; y++)
            if (frame.data[(y*xres) + x])
                goto tl_lm_corner;
tl_lm_corner: candidate1 = cv::Point_<int>(x, y);
    // find topmost
    for (y = 0; y < hyres; y++)
        for (x = 0; x < hxres; x++)
            if (frame.data[(y*xres) + x])
                goto tl_tm_corner;
tl_tm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_tl);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_tl);
    p[0] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
    

    // find bottom left corner
    // find leftmost
    for (x = 0; x < hxres; x++)
        for (y = yres-1; y >= hyres; y--)
            if (frame.data[(y*xres) + x])
                goto bl_lm_corner;
bl_lm_corner: candidate1 = cv::Point_<int>(x, y);
    // find bottommost
    for (y = yres-1; y >= hyres; y--)
        for (x = 0; x < hxres; x++)
            if (frame.data[(y*xres) + x])
                goto bl_bm_corner;
bl_bm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_bl);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_bl);
    p[1] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
    
    
    // find top right corner
    // find rightmost
    for (x = xres-1; x >= hxres; x--)
        for (y = 0; y < hyres; y++)
            if (frame.data[(y*xres) + x])
                goto tr_rm_corner;
tr_rm_corner: candidate1 = cv::Point_<int>(x, y);
    // find topmost
    for (y = 0; y < hyres; y++)
        for (x = xres-1; x >= hxres; x--)
            if (frame.data[(y*xres) + x])
                goto tr_tm_corner;
tr_tm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_tr);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_tr);
    p[2] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
    

    // find bottom right corner
    // find rightmost
    for (x = xres-1; x >= hxres; x--)
        for (y = yres-1; y >= hyres; y--)
            if (frame.data[(y*xres) + x])
                goto br_rm_corner;
br_rm_corner: candidate1 = cv::Point_<int>(x, y);
    // find bottommost
    for (y = yres-1; y >= hyres; y--)
        for (x = xres-1; x >= hxres; x--)
            if (frame.data[(y*xres) + x])
                goto br_bm_corner;
br_bm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_br);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_br);
    p[3] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
}


float Vision::getAverageDepthOfRegion (cv::Mat& depthFrame, int* rough_orientation, cv::Point_<int> tl, cv::Point_<int> br)
{
    static const int xres = depthFrame.cols;
    int left_total  = 0, left_nzero  = 0;
    int right_total = 0, right_nzero = 0;
    float left_average, right_average;
    int halfway_x = ((br.x - tl.x) / 2) + tl.x;
    int x, y, z;
    
    for (y = tl.y; y < br.y; y++)
    {
        for (x = tl.x; x < br.x; x++)
        {
            z = *((uint16_t*) (depthFrame.data + (((y * xres) + x)*2)));
            if (z != 0)
            {
                if (x < halfway_x)
                {
                    left_total += z;
                    left_nzero++;
                }
                else
                {
                    right_total += z;
                    right_nzero++;
                }
            }
        }
    }
    left_average  = ((float) left_total)  / ((float) left_nzero);
    right_average = ((float) right_total) / ((float) right_nzero);
    if (left_average < right_average)
        *rough_orientation = -1;
    else
        *rough_orientation = 1;
    
    return (left_average + right_average) / 2.0;
}


/// @brief  TODO
void Vision::projectPoints (cv::Mat& depthFrame, cv::Point_<int>* p2D, cv::Point3_<float>* p3D, int count)
{
    static const int xres = depthFrame.cols, yres = depthFrame.rows;
    static const float projection_constant = (((PIXEL_SIZE * IMAGE_WIDTH * 2) / xres) / FOCAL_LENGTH);
    int i;
    
    for (i = 0; i < count; i++)
    {
        p3D[i].z = *((uint16_t*) (depthFrame.data + (((p2D[i].y * xres) + p2D[i].x)*2)));
        p3D[i].x = (p2D[i].x - (xres/2)) * p3D[i].z * projection_constant;
        p3D[i].y = ((yres/2) - p2D[i].y) * p3D[i].z * projection_constant;
    }
}


/// @brief  TODO
float Vision::cv_euclidean_distance2 (cv::Point_<int> p1, cv::Point_<int> p2)
{
    float dx = (float) p1.x - p2.x;
    float dy = (float) p1.y - p2.y;
    return dx*dx + dy*dy;
}


/// @brief  TODO.
void Vision::kind_of_subsample (cv::Mat* dst, cv::Mat* src, int x_step, int y_step, int y_start, int y_end)
{
    int x, y, dst_i;
    int xres = src->size().width;
//    int yres = src->size().height;

    for (dst_i=0, y=y_start; y < y_end; y += y_step)
    {
        for (x=0; x < xres; x += x_step, dst_i++)
        {
            // sum all substep pixels
            int substep_total = 0;
            int substep_nzero = 0;
            for (int y_prime = 0; y_prime < y_step; y_prime++)
            {
                for (int x_prime = 0; x_prime < x_step; x_prime++)
                {
                    int v = src->data[((y + y_prime) * xres) + (x + x_prime)];
                    if (v != 0)
                    {
                        substep_nzero++;
                        substep_total += v;
                    }
                }
            }

            dst->data[dst_i] = (substep_nzero == 0) ? 0u : (uint8_t) (substep_total / substep_nzero);
        }
    }
}


bool Vision::checkForOccupancy (void)
{
    // Construct the clipping frame.
    cv::Rect clipping_frame(cv::Point_<int>(OCCUPANCY_ANALYSIS_X_START, OCCUPANCY_ANALYSIS_Y_START), cv::Point_<int>(OCCUPANCY_ANALYSIS_X_END, OCCUPANCY_ANALYSIS_Y_END));
    
    // Make a histogram of the clipped depth.
    cv::Mat clipped_depth_frame = mDepthFrame(clipping_frame);
    Histogram clipped_depth_histogram(clipped_depth_frame);
    
    // Take a cheeky peeky at the histogram to see if the chunk we want is sufficiently full of 'stuff'.
    return (clipped_depth_histogram.getRange(OCCUPANCY_ANALYSIS_RANGE_START, OCCUPANCY_ANALYSIS_RANGE_END) > OCCUPANCY_ANALYSIS_THRESHOLD);
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
