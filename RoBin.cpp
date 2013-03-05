/**
 * @file        RoBin.cpp
 * @brief       The main processing loop linking all the modules together.
 */

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include "Robot.h"
#include "Vision.h"
#include "TCPInterface.h"

#define MOVEMENT_ENABLED
#define ATOMIC_TURN_AMOUNT  20 // FPS delay
#define MOVE_SPEED         100 // mm/s
#define TURN_SPEED         100 // mm/s



void turnRobot (Robot* robot, sint32_t direction)
{
    static bool_t alreadyTurning = 0;
    
    if (alreadyTurning && direction == 0)
    {
        alreadyTurning = 0;
        robot->setSpeed(0, 0);
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
    uint8_t  bumperValues[2];
    sint8_t  panicStations = 0;
    int      retVal;
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
        // check panic situation
        panicStations = vinny.shouldWePanic();
        if (panicStations > 0 && !turnCounter)
            turnCounter =  ATOMIC_TURN_AMOUNT;
        else if (panicStations < 0 && !turnCounter)
            turnCounter = -ATOMIC_TURN_AMOUNT;
        
        // check bumpers
        reginald.getBumperValues(bumperValues);
        if (bumperValues[0] && turnCounter == 0)    // left bumper
            turnCounter =  ATOMIC_TURN_AMOUNT;
        else if (bumperValues[1] && !turnCounter)    // right bumper
            turnCounter = -ATOMIC_TURN_AMOUNT;
        
#ifdef MOVEMENT_ENABLED
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
