#include "RobotClient.h"


/// @brief  Constructor.
RobotClient::RobotClient (void) : mTCPI(TCPCLIENT, STREAMING_PORT_D),
                                  mStreamingImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3)
{
    mTCPI.waitForServerConnection();
    
    cvNamedWindow(STREAM_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
}


/// @brief  Deconstructor.
RobotClient::~RobotClient (void)
{
    cvDestroyAllWindows();
}


void RobotClient::receiveFrame (void)
{
    static uint32_t frameSize;
    
    // read frame header and grow our buffer accordingly. The line is initially filled with nulls that must be eaten.
    mTCPI.readBytes(&frameSize, 4);
    while (frameSize == 0)
        mTCPI.readBytes(&frameSize, 4);
    mStreamingBuffer.resize(frameSize);
    
    // read the frame body.
    mTCPI.readBytes(&(mStreamingBuffer.front()), frameSize);
    
    // Decode it.
    const cv::Mat bufferWrapper(mStreamingBuffer, false);
    mStreamingImage = cv::imdecode(bufferWrapper, -1);
}


int RobotClient::displayFrame (void)
{
    int retVal;
    
    if (mStreamingImage.empty())
        printf("ERROR: Image was empty, not displaying :(\n");
    else
        cv::imshow(STREAM_WINDOW_NAME, mStreamingImage);
    
    retVal = cvWaitKey(5);
    return retVal;
}

uint32_t RobotClient::getStreamingBufferSize (void) const
{
    return mStreamingBuffer.size();
}



float getDifference_s (timespec* startingTime, timespec* endingTime)
{
    time_t diff_sec  = endingTime->tv_sec  - startingTime->tv_sec;
    long   diff_nsec = endingTime->tv_nsec - startingTime->tv_nsec;
    // account for annoying 'overflow' from nsec into sec.
    if (diff_nsec < 0)
    {
        diff_sec  -= 1;
        diff_nsec += 1000000000;
    }
    
    float  diff_in_seconds = diff_sec + (diff_nsec / 1000000000.0f);
    return diff_in_seconds;
}


/// @brief  Entry point.
int main (void)
{
    int      buttonPress;
    timespec frameStart, frameEnd;
    float    timePerFrame, fps, kbps;
    uint32_t i_fps=0, i_kbps=0;
    float    CAfps=0, CAkbps=0;
    RobotClient rc;
    
    if (sizeof(uint32_t) != 4)
        printf("BIG PROBLEMS COMING YOUR WAY, KILL THE PROGRAM BEFORE IT CRASHES YOUR COMPUTER LOL\n");
    
    while (1)
    {
        clock_gettime(CLOCK_MONOTONIC, &frameStart);
        
        // bulk of the loop
        rc.receiveFrame();
        buttonPress = rc.displayFrame();
/*        if (buttonPress >= 0)
            break;*/
        
        // calculate and print fps (and other stats).
        clock_gettime(CLOCK_MONOTONIC, &frameEnd);
        timePerFrame = getDifference_s(&frameStart, &frameEnd);
        fps  = 1.0f / timePerFrame;
        kbps = (rc.getStreamingBufferSize() / 1024.0f) / timePerFrame;
        
        i_fps += 1;
        CAfps += (fps - CAfps) / (i_fps);
        i_kbps += 1;
        CAkbps += (kbps - CAkbps) / (i_kbps);
        
        
        printf("FPS: %.1f\tThroughput: %.2f kBps  \r", CAfps, CAkbps);
        fflush(stdout);
    }
    
    printf("\nExitting.\n");
    
    return 0;
}
