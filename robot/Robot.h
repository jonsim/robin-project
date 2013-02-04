/**
 * @file        Robot.h
 * @brief       Functions to control the robot.
 */
#ifndef ROBOT_H
#define ROBOT_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include "SerialInterface.h"


/*-------------------- DEFINES  --------------------*/
enum RobotMode
{
    PASSIVE = 128u,
    SAFE    = 131u,
    FULL    = 132u
};


/*---------------- CLASS DEFINITION ----------------*/
class Robot
{
public:
    Robot (void);
    ~Robot (void) {}
    
    const sint16_t getDistance (void) const;
    const sint16_t getAngle    (void) const;

    void setMode  (const RobotMode rm);
    void setSpeed (const sint16_t lVel, const sint16_t rVel);
    void setLEDs  (const bool playLED, const bool advanceLED);
    
    void printCharging (void) const;
    void printStatus   (void) const;

private:
    SerialInterface mSI;
};

#endif
