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
    OFF     = 0u,
    PASSIVE = 128u,
    SAFE    = 131u,
    FULL    = 132u
};


/*--------------------  MACROS  --------------------*/
// Ensures the robot currently has sufficient permissions to perform the action requested
#define CHECK_ROBOTMODE(requiredMode)                                       \
    if (mCurrentMode < requiredMode)                                        \
    {                                                                       \
        printf("WARNING: Not in required mode to perform that action.\n");  \
    }


/*---------------- CLASS DEFINITION ----------------*/
class Robot
{
public:
    Robot (void);
    ~Robot (void);
    
    void           getCliffValues     (uint8_t* r) const;
    void           getBumperValues    (uint8_t* r) const;
    void           getWheelDropValues (uint8_t* r) const;
    void           getBeaconValues    (uint8_t* r) const;
    void           getButtonValues    (uint8_t* r) const;
    const uint16_t getWallSignal      (void)       const;
    const sint16_t getDistance        (void)       const;
    const sint16_t getAngle           (void)       const;
    
    void setMode     (const RobotMode rm);
    void setSpeed    (const sint16_t lVel, const sint16_t rVel);
    void setLEDs     (const bool playLED, const bool advanceLED);
    
    void targetRotation (const sint16_t degreesClockwise);
    void targetDistance (const sint16_t distance);
    
    void startDemo (const uint8_t demo_number);
    void stopDemo  (void);
    
    void printChargingStatus (void) const;
    void printBatteryStatus  (void) const;

private:
    SerialInterface* mSI;
    RobotMode        mCurrentMode;
};

#endif
