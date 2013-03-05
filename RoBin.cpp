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
    // function variables
    bool_t clientConnected = false;
    uint8_t bumperValues[2];
    int retVal;
    // module objects
    Robot  reginald;
    Vision vinny;
    TCPInterface tim(TCPSERVER, STREAMING_PORT_D);
    
    // set the robot to safe mode (allowing us to move).
    reginald.setMode(SAFE);
    
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
        
        // Robot behaviour
        vinny.shouldWePanic();
        
        reginald.getBumperValues(bumperValues);
        if (bumperValues[0])
            printf("left bump!\n");
        if (bumperValues[1])
            printf("right bump!\n");
        /*if (depthErrors > 180000)
        {
            reginald.setSpeed(-500, 500);
            msleep(250);
        }
        else
        {
            reginald.setSpeed(100, 100);
        }*/
        
        // Print stats
        printf("FPS=%.1f  \r", vinny.getFPS());
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
