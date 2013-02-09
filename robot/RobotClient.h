/**
 * @file        RobotClient.h
 * @brief       Functions to receive data from the Robot and display it.
 */
#ifndef ROBOTCLIENT_H
#define ROBOTCLIENT_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include "TCPInterface.h"
// fps calculation
#include <time.h>
// opencv
#include <cv.h>
#include <highgui.h>


/*-------------------- DEFINES  --------------------*/
#define STREAM_WINDOW_NAME "Streaming Video"


/*--------------------  MACROS  --------------------*/
// None.


/*---------------- CLASS DEFINITION ----------------*/
class RobotClient
{
public:
    RobotClient (void);
    ~RobotClient (void);
    
    void receiveFrame (void);
    int  displayFrame (void);
    
    uint32_t getStreamingBufferSize (void) const;


private:
    // Client interface.
    TCPInterface mTCPI;
    
    // OpenCV image containers.
    cv::Mat              mStreamingImage;
    std::vector<uint8_t> mStreamingBuffer;
};

#endif
