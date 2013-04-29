/**
 * @file        Robot.h
 * @brief       Functions to control the robot.
 */
#ifndef ROBOT_H
#define ROBOT_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include "SerialInterface.h"
#include "GridMap.h"
#include <time.h>


/*-------------------- DEFINES  --------------------*/
// pathing defines
#define PATHING_MARKER_STOPPING_DISTANCE  800 // mm. the distance to stop away from markers.
#define PATHING_MAX_MOVE_DISTANCE        2000 // mm.
#define PATHING_ANGULAR_STD_DEV            20 // degrees.
#define PATHING_NAP_DURATION             2 // the amount of time, in seconds, to wait for scary things to go away.
#define ROBOT_MOVE_SPEED                  200 // mm/s
#define ROBOT_TURN_SPEED                  200 // mm/s

enum RobotMode
{
    OFF     = 0u,
    PASSIVE = 128u,
    SAFE    = 131u,
    FULL    = 132u
};

enum PathingType
{
    RANDOM,
    GREEDY_NODE,
    GREEDY_TABLE,
    DIRECT_ORDER
};

struct PathingAction
{
    PathingType type;
    Point2i     target;
    int         first_angle;
    int         displacement;
    int         final_angle;
    
    PathingAction (void) : type(RANDOM), target(0,0), first_angle(0), displacement(0), final_angle(0) {}
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
    
    void timestep (sint8_t object_avoidance, bool target_recognition, MarkerData& target_recognition_data);
    
    void updateMap (void);
    
    PathingAction generateRandomPathingAction (int rotation_mean=0);
    PathingAction generateMarkerPathingAction (MarkerData& marker_data);
    void executePathingAction (PathingAction& action);


private:
    PathingType generatePathingType (void);
    float randNormallyDistributed (float mu, float sigma);
    
    void updateTargets (int new_move, int new_turn);
    void updateAccumulators (int d_move, int d_turn);
    void zeroTargetsAndAccumulators (void);

    SerialInterface* mSI;
    RobotMode        mCurrentMode;
    GridMap          mMap;
    int  mMoveAccumulator;
    int  mMoveTarget;
    int  mTurnAccumulator;
    int  mTurnTarget;
    bool mIsNapping;
    time_t mNapStarted;
    bool mIsInterruptable;
    bool mIsIdle;
};

#endif
