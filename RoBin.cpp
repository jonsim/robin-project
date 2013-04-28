/**
 * @file        RoBin.cpp
 * @brief       The main processing loop linking all the modules together.
 */

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include "Robot.h"
#include "Vision.h"
#include "TCPInterface.h"

//#define MOVEMENT_ENABLED
#define ATOMIC_TURN_AMOUNT  10 // FPS delay
#define MOVE_SPEED         200 // mm/s
#define TURN_SPEED         200 // mm/s



void turnRobot (Robot* robot, sint32_t direction)
{
    static bool_t alreadyTurning = 0;
    
    if (alreadyTurning && direction == 0)
    {
        alreadyTurning = 0;
        robot->setSpeed(0, 0);
        robot->updateMap();
    }
    else if (!alreadyTurning && direction > 0)
    {
        alreadyTurning = 1;
        robot->setSpeed(TURN_SPEED, -TURN_SPEED);
    }
    else if (!alreadyTurning && direction < 0)
    {
        alreadyTurning = 1;
        robot->setSpeed(-TURN_SPEED, TURN_SPEED);
    }
}

void moveRobot (Robot* robot, sint32_t direction)
{
    static bool_t alreadyMoving = 0;
    
    if (alreadyMoving && direction == 0)
    {
        alreadyMoving = 0;
        robot->setSpeed(0, 0);
        robot->updateMap();
    }
    else if (!alreadyMoving && direction > 0)
    {
        alreadyMoving = 1;
        robot->setSpeed(MOVE_SPEED, MOVE_SPEED);
    }
    else if (!alreadyMoving && direction < 0)
    {
        alreadyMoving = 1;
        robot->setSpeed(-MOVE_SPEED, -MOVE_SPEED);
    }
}




/*-------------------- MAIN FUNCTION --------------------*/
/// @brief  Unit testing.
int main (void)
{
    // function variables
    bool_t   clientConnected = false;
    sint32_t turnCounter = 0;
    sint8_t  panicStations = 0;
    int      retVal;
    // module objects
#ifdef MOVEMENT_ENABLED
    Robot    reginald;
    uint8_t  bumperValues[2];
#endif
    Vision vinny;
    TCPInterface tim(TCPSERVER, STREAMING_PORT_D);
    
#ifdef MOVEMENT_ENABLED
    // set the robot to safe mode (allowing us to move).
    reginald.setMode(SAFE);
#endif
    
    printf("starting camera loop...\n");
    while (1)
    {
        // First check for exit conditions.
        if (_kbhit())
            break;
        
        // Check for clients trying to connect. This currently blocks but it could not.
#if defined(DEPTH_STREAMING_ENABLED) || defined(COLOR_STREAMING_ENABLED)
        if (!clientConnected)
            clientConnected = tim.checkForClients();
#endif
        
        // Sample the camera data
        vinny.captureFrame();
        
        // Robot behaviour
        // Check the camera data for things that will make us panic
        panicStations = vinny.checkForObstacles();
        if (panicStations > 0 && !turnCounter)
            turnCounter =  ATOMIC_TURN_AMOUNT;
        else if (panicStations < 0 && !turnCounter)
            turnCounter = -ATOMIC_TURN_AMOUNT;
        
#ifdef MOVEMENT_ENABLED
        // check cliff sensors
        //   done automatically in safe mode
        
        // check bumpers
        reginald.getBumperValues(bumperValues);
        if (bumperValues[0] && turnCounter == 0)    // left bumper
            turnCounter =  ATOMIC_TURN_AMOUNT;
        else if (bumperValues[1] && !turnCounter)    // right bumper
            turnCounter = -ATOMIC_TURN_AMOUNT;
        
        // move robot
        if (turnCounter == 0)
        {
            turnRobot(&reginald, 0);
            moveRobot(&reginald, 1);
        }
        else if (turnCounter > 0)
        {
            moveRobot(&reginald, 0);
            turnRobot(&reginald, turnCounter--);
        }
        else if (turnCounter < 0)
        {
            moveRobot(&reginald, 0);
            turnRobot(&reginald, turnCounter++);
        }
#endif
        
        // Print stats
        printf("FPS: %.1f \tpanicStations: %d \r", vinny.getFPS(), panicStations);
        fflush(stdout);
        
        // If there's a client connected send the depth data to them.
#if defined(DEPTH_STREAMING_ENABLED) || defined(COLOR_STREAMING_ENABLED)
        if (clientConnected)
        {
    #ifdef DEPTH_STREAMING_ENABLED
            vinny.compressDepthFrame();
    #else
            vinny.compressColorFrame();
    #endif
            uint32_t frameSize = (uint32_t) vinny.mStreamBuffer.size();
            retVal = tim.writeBytes(&frameSize, 4);
            retVal = tim.writeBytes(&(vinny.mStreamBuffer.front()), frameSize);
            if (retVal < 0)
                break;
        }
#endif
    }
//    vinny.compressFrameToDisk("depthFrame.png");
    
    printf("\nExitting.\n");
    
    return 0;
}
