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
#include "Vision.h"
#include <time.h>
#include <queue>


/*-------------------- DEFINES  --------------------*/
// pathing defines
#define PATHING_MARKER_STOPPING_DISTANCE  800 // mm. the distance to stop away from markers.
#define PATHING_MAX_MOVE_DISTANCE        2000 // mm.
#define PATHING_ANGULAR_STD_DEV            40 // degrees.
#define PATHING_NAP_DURATION                1 // the amount of time, in seconds, to wait for scary things to go away.
#define TARGET_NAP_DURATION                 3 // the amount of time, in seconds, to wait at tables.
#define ROBOT_MOVE_SPEED                  200 // mm/s
#define ROBOT_TURN_SPEED                  100 // mm/s
#define PATHING_MAX_PATHING_ACTIONS        50
//#define WHEEL_MOTOR_EMULATION
#define WHEEL_MOTOR_EMULATOR_DISTANCE      50
#define WHEEL_MOTOR_EMULATOR_ANGLE          5
#define VERBOSE_PRINTOUTS
//#define VERY_VERBOSE_PRINTOUTS

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
    DIRECT_ORDER,
    NONE
};
enum ActionPriority
{
    LOWEST = 0,
    LEVEL1 = 1,
    LEVEL2 = 2,
    HIGHEST = 3
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

enum MotorActionType
{
    ROTATION,
    TRANSLATION
};
struct MotorAction
{
    MotorActionType type;
    ActionPriority  priority;
    union
    {
        int angle;
        int displacement;
    } action;
    
    MotorAction (MotorActionType type_init, int value_init, ActionPriority priority_init) : type(type_init), priority(priority_init)
    {
        if (type == ROTATION)
            action.angle = value_init;
        else
            action.displacement = value_init;
    }
    
    bool operator< (const MotorAction& other)
    {
        return (priority < other.priority);
    }
    bool operator> (const MotorAction& other)
    {
        return (priority > other.priority);
    }
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
    Robot (const char* map_file = NULL);
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
    
    bool isTurning (void);
    
    void printChargingStatus (void) const;
    void printBatteryStatus  (void) const;
    
    void timestep (Vision* vision, sint8_t object_avoidance, bool target_recognition, MarkerData& target_recognition_data);
    
//    void updateMap (void);
    
//    void executePathingAction (PathingAction& action);


private:
    PathingType   generateRandomPathingType   (void);
    PathingAction generateRandomPathingAction (int rotation_mean=0);
    PathingAction generateN2NPathingAction (const uint32_t start, const uint32_t end);
    PathingAction generateN2NPathingAction (const Point2i& start, const Point2i& end);
    PathingAction generateMarkerPathingAction (MarkerData& marker_data);
    
    void updateTargets (int new_move, int new_turn);
    void updateAccumulators (int d_move, int d_turn);
    void zeroTargetsAndAccumulators (void);
    
    void startNapping (int duration);
    bool nappingTimeUp (void);
    
    void processMotorActions (Vision* vision);
    void executePathingAction (void);
    void reroutePathingActions (void);
    void dropPathingActions (void);

    SerialInterface* mSI;
    RobotMode        mCurrentMode;
    GridMap          mMap;
    // accumulators
    int  mMoveAccumulator;
    int  mMoveTarget;
    int  mTurnAccumulator;
    int  mTurnTarget;
    // napping stuff
    bool   mIsNapping;
    time_t mNapStarted;
    int    mNapDuration;
    // pathing states
//    bool mIsInterruptable;
    bool mIsIdle;
//    PathingType               mCurrentActionType;
    PathingAction             mCurrentAction;
    std::queue<MotorAction>   mMotorActions;
    std::queue<PathingAction> mPathingActions;
};

#endif
