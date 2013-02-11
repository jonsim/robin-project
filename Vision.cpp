#include "Vision.h"


/// @brief  Constructor.
Vision::Vision (void) : mStreamingDepthRaw(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3)
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
    retVal = mContext.FindExistingNode(XN_NODE_TYPE_DEPTH, mDepthGenerator);
    CHECK_RETURN_XN(retVal, "Find depth generator");
    retVal = mContext.FindExistingNode(XN_NODE_TYPE_IMAGE, mColorGenerator);
    CHECK_RETURN_XN(retVal, "Find image generator");
    retVal = xnFPSInit(&mXnFPS, 180);
    CHECK_RETURN_XN(retVal, "FPS Init");
    printf("done.\n");
}


/// @brief  Shuts down the camera, releasing all objects associated with it.
void Vision::shutdownCamera (void)
{
    mDepthGenerator.Release();
    mColorGenerator.Release();
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
    CHECK_RETURN_XN(retVal, "UpdateData failed. We could just continue here if this becomes a problem.");

    // Read the data into our containers.
    mDepthGenerator.GetMetaData(mDepthMetaData);
    mDepthData = (const uint16_t*) mDepthMetaData.Data();
    
/*#ifdef COLOR_STREAM
    color.GetMetaData(colorMD);
    const XnRGB24Pixel* colorData = colorMD.RGB24Data();
    retVal = write(clientSocketC, colorData, colorBufferLen);
    if (retVal < 0)
        break;
#endif*/
}


/// @brief  Compresses the currently saved depth frame (loaded from the captureFrame() method) to a
///         JPEG which resides in the mStreamingDataJPEG object variable.
const std::vector<uint8_t>* Vision::compressFrame (void)
{
    static int paramsArray[] = {CV_IMWRITE_JPEG_QUALITY, COMPRESSION_QUALITY};
    static std::vector<int> paramsVector(paramsArray, paramsArray + sizeof(paramsArray) / sizeof(int));
    
    // Produce a human-viewable colour representation of the depth data
    createColourDepthImage(&mStreamingDepthRaw, mDepthData);
    
    // Compress it to a JPEG
    cv::imencode(".JPEG", mStreamingDepthRaw, mStreamingDepthJPEG, paramsVector);
    
    // Stream it
    //retVal = write(clientSocketD, &(mStreamingDepthJPEG.size()), 4);
    //CHECK_RETURN(retVal, "streamFrame socket write");
    //retVal = write(clientSocketD, &(mStreamingDepthJPEG.front()), mStreamingDepthJPEG.size());
    //CHECK_RETURN(retVal, "streamFrame socket write");
    
    // return it
    return &mStreamingDepthJPEG;
}


/// @brief  Compresses the currently saved depth frame (loaded from the captureFrame() method) to a
///         JPEG which resides in the mStreamingDataJPEG object variable.
void Vision::compressFrameToDisk (const char* filename)
{
    static int paramsArray[] = {CV_IMWRITE_JPEG_QUALITY, COMPRESSION_QUALITY};
    static std::vector<int> paramsVector(paramsArray, paramsArray + sizeof(paramsArray) / sizeof(int));
    
    // Produce a human-viewable colour representation of the depth data
    createColourDepthImage(&mStreamingDepthRaw, mDepthData);
    
    // Compress it to a JPEG
    cv::imwrite(filename, mStreamingDepthRaw, paramsVector);
    
    // Stream it
    //retVal = write(clientSocketD, &(mStreamingDepthJPEG.size()), 4);
    //CHECK_RETURN(retVal, "streamFrame socket write");
    //retVal = write(clientSocketD, &(mStreamingDepthJPEG.front()), mStreamingDepthJPEG.size());
    //CHECK_RETURN(retVal, "streamFrame socket write");
}


float Vision::getFPS (void)
{
    xnFPSMarkFrame(&mXnFPS);
    return xnFPSCalc(&mXnFPS);
}

void Vision::buildDepthHistogram (void)
{
    uint16_t x, y;
    uint32_t one_step_offset, i;
    
    // zero the histogram
    for (i = 0; i < MAX_DEPTH; i++)
        mDepthHistogram[i] = 0;
    
    // count the values
    for (y = 0, one_step_offset = 0; y < IMAGE_HEIGHT; y++)
        for (x = 0; x < IMAGE_WIDTH; x++, one_step_offset++)
            mDepthHistogram[mDepthData[one_step_offset]]++;
}

uint32_t Vision::queryDepthHistogram (uint16_t v)
{
    return (v < MAX_DEPTH) ? mDepthHistogram[v] : 0;
}



/// @brief  Creates a colour image (as a cv::Mat) representation of the depth data in the given array.
/// @param  dst   The cv::Mat into which the colour representation will be saved. The size of this
///               is defined by the IMAGE_HEIGHT and IMAGE_WIDTH values. The cv::Mat needs to be
///               in the CV_8UC3 mode (8 bit RGB).
/// @param  src   A pointer to the uint16_t array of depth data. The size of this is defined by the
///               IMAGE_HEIGHT and IMAGE_WIDTH values.
void Vision::createColourDepthImage (cv::Mat* dst, const uint16_t* src)
{
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
}


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
