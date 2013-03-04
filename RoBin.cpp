/**
 * @file        RoBin.cpp
 * @brief       The main processing loop linking all the modules together.
 */

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include "Robot.h"
#include "Vision.h"
#include "TCPInterface.h"




/*-------------------- MAIN FUNCTION --------------------*/
/// @brief  Unit testing.
int main (void)
{
    Robot  reginald;
    Vision vinny;
    TCPInterface tim(TCPSERVER, STREAMING_PORT_D);
    bool_t clientConnected = false;
    uint32_t depthErrors=0;
    int retVal;
    
    reginald.setMode(SAFE);
    
    if (sizeof(uint32_t) != 4)
        printf("BIG PROBLEMS COMING YOUR WAY, KILL THE PROGRAM BEFORE IT CRASHES YOUR COMPUTER LOL\n");
    printf("starting camera loop...\n");
    
    while (1)
    {
        // First check for exit conditions.
        if (_kbhit())
            break;
        
        // Check for clients trying to connect
        // TODO this currently blocks and it shoudln't ;(
        if (!clientConnected)
            clientConnected = tim.checkForClients();
        
        // Sample the camera data
        vinny.captureFrame();
        vinny.buildDepthHistogram();
        depthErrors = vinny.queryDepthHistogram(0u);
        
        // Robot behaviour
        if (depthErrors > 180000)
        {
            reginald.setSpeed(-500, 500);
            msleep(250);
        }
        else
        {
            reginald.setSpeed(100, 100);
        }
        
        // Print stats
        printf("FPS=%.1f  \tHist[0]=%d  \r", vinny.getFPS(), vinny.queryDepthHistogram(0u));
        fflush(stdout);
        
        // If there's a client connected send the depth data to them.
        if (clientConnected)
        {
            const std::vector<uint8_t>* streamBuffer = vinny.compressFrame();
            uint32_t frameSize = (uint32_t) streamBuffer->size();
            retVal = tim.writeBytes(&frameSize, 4);
            retVal = tim.writeBytes(&(streamBuffer->front()), frameSize);
            if (retVal < 0)
                break;
        }
    }
    
    printf("Exitting.\n");
    
    return 0;
}
