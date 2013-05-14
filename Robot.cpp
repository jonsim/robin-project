#include "Robot.h"


/// @brief  Constructor. Opens and initialises the serial connection before turning on the Robot. It
///         starts in PASSIVE mode. See setMode() for further information.
Robot::Robot (const char* map_file) : mCurrentMode(OFF),
                                      mMap(map_file),
                                      mMoveAccumulator(0),
                                      mMoveTarget(0),
                                      mTurnAccumulator(0),
                                      mTurnTarget(0),
                                      mCurrentLeftVelocity(0),
                                      mCurrentRightVelocity(0),
                                      mIsNapping(false),
                                      mIsIdle(true)
{
    mSI = new SerialInterface();
    mSI->start();
    mSI->writeByte(128u);
    mCurrentMode = PASSIVE;
    srand(time(NULL));
}


/// @brief  Deconstructor. Ensures the robot is in a safe state to exit (to avoid exitting the
///         program and having the thing continue moving).
Robot::~Robot (void)
{
    setSpeed(0, 0);
}


/// @brief  Gets the boolean values of the cliff sensors (stored as uint8_t for efficiency).
/// @param  The array into which the boolean cliff values will be stored. This must be 4 elements
///         long (or more). The values are stored as: [0] = Front Left, [1] = Front Right,
///         [2] = Centre Left, [3] = Centre Right.
void Robot::getCliffValues (uint8_t* r) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[4][2] = {{142u, 10u},   // returns front  left  state
                             {142u, 11u},   // returns front  right state
                             {142u,  9u},   // returns centre left  state
                             {142u, 12u}};  // returns centre right state
    
    for (uint8_t i = 0u; i < 4u; i++)
    {
        mSI->writeBytes(command[i], sizeof(command[i]));
        r[i] = mSI->readByte();
    }
}


/// @brief  Gets the boolean values of the front bumper (stored as uint8_t for efficiency).
/// @param  The array into which the boolean bumper values will be stored. This must be 2 elements
///         long (or more). The values are stored as: [0] = Left, [1] = Right. A value of 0 means
///         no bumper activation, a value of 1 means bumper activation.
void Robot::getBumperValues (uint8_t* r) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 7u};
    uint8_t response;
    
    mSI->writeBytes(command, 2);
    response = mSI->readByte();
    
    r[0] = response & 2u;
    r[1] = response & 1u;
}


/// @brief  Gets the boolean values of the wheel drop sensors (stored as uint8_t for efficiency).
/// @param  The array into which the boolean wheel drop values will be stored. This must be 3
///         elements long (or more). The values are stored as [0] = Left, [1] = Right, [2] = Caster.
void Robot::getWheelDropValues (uint8_t* r) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 7u};
    uint8_t response;
    
    mSI->writeBytes(command, sizeof(command));
    response = mSI->readByte();
    
    r[0] = response &  8u;
    r[1] = response &  4u;
    r[2] = response & 16u;
}


/// @brief  Gets the boolean values of the docking beacon beams (stored as uint8_t for efficiency).
/// @param  The array into which the boolean docking beacon values will be stored. This must be 3
///         elements long (or more). The values are stored as: [0] = Red Buoy, [1] = Green Buoy,
///         [2] = Force Field.
void Robot::getBeaconValues (uint8_t* r) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 17u};
    uint8_t response;
    
    mSI->writeBytes(command, sizeof(command));
    response = mSI->readByte();
    
    if ((response & 240u) != 240u)
        response = 0u;
    
    r[0] = response & 8u;    // red
    r[1] = response & 4u;    // green
    r[2] = response & 2u;    // forcefield
}


/// @brief  Gets the boolean values of the buttons (stored as uint8_t for efficiency).
/// @param  The array into which the boolean button values will be stored. This must be 2 elements
///         long (or more). The values are stored as: [0] = Play Button, [1] = Advance Button.
void Robot::getButtonValues (uint8_t* r) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 18u};
    uint8_t response;
    
    mSI->writeBytes(command, sizeof(command));
    response = mSI->readByte();
    
    r[0] = response & 1u;
    r[1] = response & 4u;
}


/// @brief  Gets the strength of the wall sensor.
/// @return The wall sensor's signal. Range: 0 - 4095.
const uint16_t Robot::getWallSignal (void) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 27u};
    uint8_t response[2];
    
    mSI->writeBytes(command, sizeof(command));
    mSI->readBytes(response, sizeof(response));
    
    return make_uint16_t(response[0], response[1]);
}


/// @brief  Gets the distance that the Robot has traveled in millimeters since the time it was last
///         requested. This is the same as the sum of the distance traveled by both wheels divided
///         by two. Positive values indicate forward travel; negative values indicate reverse
///         travel. If the value is not polled frequently enough, it is capped at its min/max.
/// @return The distance travelled. Range: -32768 - 32767.
const sint16_t Robot::getDistance (void) const
{
#ifndef WHEEL_MOTOR_EMULATION
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 19u};
    uint8_t response[2];
    sint16_t r;
    
    if (mMoveTarget == 0)
        return 0;
    
    usleep(10000);
    mSI->writeBytes(command, 2);
    mSI->readBytes(response, 2);
    
    if (mMoveTarget == 0)
        return 0;
    // override because the roomba is a piece of crap
    /*if (mMoveTarget < 0)
        return -30;
    if (mMoveTarget > 0)
        return 30;
    return 0;*/
    
    r = make_sint16_t(response[0], response[1]);
    if (r > 200 || r < -200)
    {
        if (mMoveTarget > 0)
            r = 20;
        else
            r = -20;
    }
    printf("distance travelled = %d (%x %x)\n", r, response[0], response[1]);
/*    if (r > ROBOT_MOVE_SPEED || r < -ROBOT_MOVE_SPEED) // something's gone wrong, the roomba really isn't very good at this...
    {
        // try flipping them round, possibly it's cocked up and sent them out of order
        r = make_sint16_t(response[1], response[0]);
        
        // is it still wrong?
        if (r > ROBOT_MOVE_SPEED || r < -ROBOT_MOVE_SPEED)
        {
            // if it is just return a kind of sane number based on the FPS (hardcoded because why not). this is a disgusting solution to the problem.
            r = ROBOT_MOVE_SPEED/9;
        }
    }
    */
    return r;
#else
    if (mMoveTarget < 0)
        return -WHEEL_MOTOR_EMULATOR_DISTANCE;
    if (mMoveTarget > 0)
        return WHEEL_MOTOR_EMULATOR_DISTANCE;
    return 0;
#endif
}


/// @brief  Gets the angle in degrees that the Robot has turned since the angle was last requested.
///         Counter-clockwise angles are positive and clockwise angles are negative. If the value is
///         not polled frequently enough, it is capped at its min/max. 
/// @return The angle turned. Range: -32768 - 32767.
const sint16_t Robot::getAngle (void) const
{
#ifndef WHEEL_MOTOR_EMULATION
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 20u};
    uint8_t response[2];
    sint16_t r;
    
    usleep(10000);
    mSI->writeBytes(command, 2);
    mSI->readBytes(response, 2);
    
    if (mTurnTarget == 0)
        return 0;
    
    /*
    // override because the roomba is a piece of crap
    if (mTurnTarget > 0)
        return 12;
    else if (mTurnTarget < 0)
        return -12;
    return 0;
    */
    
    r = make_sint16_t(response[0], response[1]);
    printf("angle travelled = %d (%x %x)\n", r, response[0], response[1]);
    if (r > 90 || r < -90)
    {
        if (mTurnTarget > 0)
            r = 10;
        else
            r = -10;
    }
    /*
    if (r > 45 || r < -45) // something's gone wrong, the roomba really isn't very good at this...
    {
        // try flipping them round, possibly it's cocked up and sent them out of order
        r = make_sint16_t(response[1], response[0]);
        
        // is it still wrong?
        if (r > 45 || r < -45)
        {
            // if it is just return a kind of sane number based on the FPS (hardcoded because why not). this is a disgusting solution to the problem.
            r = (response[0] & 0x80) ? -15 : 15;
        }
    }*/
    
    return r;
#else
    if (mTurnTarget < 0)
        return -WHEEL_MOTOR_EMULATOR_ANGLE;
    if (mTurnTarget > 0)
        return WHEEL_MOTOR_EMULATOR_ANGLE;
    return 0;
#endif
}


/// @brief  Sets the robots mode (which determines the level of control the program can have over it
///         at the expense of automated safety features).
/// @param  rm  The mode to change to. This can be either PASSIVE, SAFE or FULL.
///             PASSIVE mode: The Robot will respond to requests for sensor data however you may not
///               change the motors/speaker/lights/low-side-drivers/digital outputs. The Robot may
///               also charge the battery and perform any of the demos.
///             SAFE mode: The Robot turns off all LEDs and can be controlled, however has internal
///               cliff sensors, wheel drop sensors and charging sensors which are enabled. Thus the
///               Robot will stop if it is plugged in or encounters a cliff or wheel drop.
///             FULL mode: Acts as safe mode but disables internal safety features.
void Robot::setMode (const RobotMode rm)
{
    mSI->writeByte(rm);
    mCurrentMode = rm;
    msleep(250);
    
    if (rm != PASSIVE)
    {
        // read the tachometry to zero out the sensors
        getDistance();
        getAngle();
        
        zeroTargetsAndAccumulators();
    }
}


/// @brief  Sets the speed of the Robot's wheels.
/// @param  lVel  The velocity of the left  wheel in mm/s. Range = -500 - 500.
/// @param  rVel  The velocity of the right wheel in mm/s. Range = -500 - 500.
void Robot::setSpeed (const sint16_t lVel, const sint16_t rVel)
{
#ifndef WHEEL_MOTOR_EMULATION
//    static sint16_t curr_lVel=0, curr_rVel=0;
    CHECK_ROBOTMODE(SAFE);
    
    if (lVel != mCurrentLeftVelocity || rVel != mCurrentRightVelocity)
    {
        mCurrentLeftVelocity  = lVel;
        mCurrentRightVelocity = rVel;
        uint8_t command[5] = {145u,
                              (uint8_t) (lVel >> 8), (uint8_t) (lVel & 0xFF),
                              (uint8_t) (rVel >> 8), (uint8_t) (rVel & 0xFF) };
        mSI->writeBytes(command, 5);
    }
#endif
}


void Robot::pause (void)
{
    mLeftVelocityBeforePause  = mCurrentLeftVelocity;
    mRightVelocityBeforePause = mCurrentRightVelocity;
    
    setSpeed(0, 0);
}


void Robot::unpause (void)
{
    setSpeed(mLeftVelocityBeforePause, mRightVelocityBeforePause);
}


/// @brief  Rotates the Robot a given angle. THIS RESETS THE ANGLE ACCUMULATOR READ BY getAngle().
/// @param  dc  The required rotation in degrees clockwise. A negative value represents an 
///             anti-clockwise rotation.
void Robot::targetRotation (const sint16_t dc)
{
#ifndef WHEEL_MOTOR_EMULATION
    CHECK_ROBOTMODE(SAFE);
    sint16_t ndc = -dc;
    
    uint8_t command_header[2] = {152u, 8u};
    uint8_t command_rotate[5] = {137u, 0xFF, 0x9C, (dc < 0) ? 0x00 : 0xFF, (dc < 0) ? 0x01 : 0xFF};
    uint8_t command_wait[3]   = {157u, (uint8_t) (ndc >> 8), (uint8_t) (ndc & 0xFF)};
    
    printf("targetting %d (0x%x, ndc>>8=%x) degree counter-clockwise rotation.\n", ndc, ndc, (uint8_t) (ndc>>8));
    
    mSI->writeBytes(command_header, sizeof(command_header));
    mSI->writeBytes(command_rotate, sizeof(command_rotate));
    mSI->writeBytes(command_wait,   sizeof(command_wait  ));
    mSI->writeByte(153u);
#endif
}


/// @brief  Sets the status of the Robot's LEDs.
/// @param  playLED     Whether or not the playLED will be lit.
/// @param  advanceLED  Whether or not the advanceLED will be lit.
void Robot::setLEDs (const bool playLED, const bool advanceLED)
{
    CHECK_ROBOTMODE(SAFE);
    uint8_t command[4] = {139u, 0u, 0u, 255u};
    if (playLED)
        command[1] |= 2u;
    if (advanceLED)
        command[1] |= 8u;
    mSI->writeBytes(command, sizeof(command));
}


/// @brief  Starts the Robot performing the currently requested demo.
/// @param  demo_number  The id of the demo to perform. This can be:
///                      0 - Cover - Covers an area using a mixture of techniques.
///                      1 - Cover & Dock - As Cover, except docking if it comes into the home base.
///                      2 - Spot Cover - Spirals outward and then inward.
///                      3 - Mouse - Drives to a wall and then follows.
///                      4 - Figure Eight - Drive in a figure of eight pattern.
///                      5 - Wimp - Drives forward when pushed and backs away from obstacles.
///                      6 - Home - Spins to locate a virtual wall and then drives towards it.
///                      7 - Tag - As Home, but continuing to the next wall when one is reached.
///                      8 - Pachelbel - Plays Pachelbel's cannon when a cliff sensor is activated.
///                      9 - Banjo - Plays a chord (selected by the bumper) from the cliff sensors.
void Robot::startDemo (const uint8_t demo_number)
{
#ifndef WHEEL_MOTOR_EMULATION
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {137u, demo_number};
    
    mSI->writeBytes(command, sizeof(command));
#endif
}


/// @brief  Halts the currently running demo. Does nothing if there is not a demo currently running.
void Robot::stopDemo (void)
{
#ifndef WHEEL_MOTOR_EMULATION
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {137u, 255u};
    
    mSI->writeBytes(command, sizeof(command));
#endif
}


/// @brief  Outputs the current charging status to stdout.
void Robot::printChargingStatus (void) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 21u};
    uint8_t response;
    const char* status;
    
    mSI->writeBytes(command, sizeof(command));
    response = mSI->readByte();
    
    switch (response)
    {
        case 0:  status = "not charging";             break;
        case 1:  status = "reconditioning charging";  break;
        case 2:  status = "full charging";            break;
        case 3:  status = "trickle charging";         break;
        case 4:  status = "waiting";                  break;
        case 5:  status = "charging fault condition"; break;
        default: status = "failed to read status";    break;
    }
    printf("Charging status: %s.\n", status);
}


/// @brief  Outputs the battery's status to stdout.
void Robot::printBatteryStatus (void) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[5][2] = {{142u, 22u},  // returns 2 byte unsigned voltage in mV
                             {142u, 23u},  // returns 2 byte signed   current in mA
                             {142u, 24u},  // returns 1 byte signed   temperature in 'C
                             {142u, 25u},  // returns 2 byte unsigned charge in mAh
                             {142u, 26u}}; // returns 2 byte unsigned capacity in mAh
    uint8_t response[2];
    
    // get voltage
    mSI->writeBytes(command[0], sizeof(command[0]));
    mSI->readBytes(response, sizeof(response));
    uint16_t voltage = make_uint16_t(response[0], response[1]);
    
    // get current
    mSI->writeBytes(command[1], sizeof(command[1]));
    mSI->readBytes(response, sizeof(response));
    sint16_t current = make_sint16_t(response[0], response[1]);
    
    // get temperature
    mSI->writeBytes(command[2], sizeof(command[2]));
    sint8_t temperature = make_sint8_t(mSI->readByte());
    
    // get charge
    mSI->writeBytes(command[3], sizeof(command[3]));
    mSI->readBytes(response, sizeof(response));
    uint16_t charge = make_uint16_t(response[0], response[1]);
    
    // get capacity
    mSI->writeBytes(command[4], sizeof(command[4]));
    mSI->readBytes(response, sizeof(response));
    uint16_t capacity = make_uint16_t(response[0], response[1]);
    
    // print
    printf("Battery status: %d mV, %d mA, %d'C, %d / %d mAh.\n", voltage, current, temperature, charge, capacity);
}


bool Robot::isTurning (void)
{
    return (mTurnTarget != 0);
}


/*
/// @brief  Updates the Robot's internal map with the latest actuator readings. It is strongly
///         recommended to call this every time a new movement instruction is issued.
void Robot::updateMap (void)
{
    const sint16_t latestDistance = getDistance();
    const sint16_t latestAngle = getAngle();
    mMap.addRelativeReadings(latestDistance, latestAngle);
}*/


void Robot::updateTargets (int new_move, int new_turn)
{
#ifdef VERBOSE_PRINTOUTS
    printf("-------- updating targets (%d, %d) --------\n", new_move, new_turn);
#endif
    if (new_move && new_turn)
        printf("WARNING: Tried to update targets with two non-zero values (n_move=%d, n_turn=%d)\n", new_move, new_turn);
    mMoveTarget = new_move;
    mTurnTarget = new_turn;
    mIsIdle = false;
}


void Robot::updateAccumulators (int d_move, int d_turn)
{
#ifdef VERY_VERBOSE_PRINTOUTS
    printf("  updating accumulators [(%d+%d)/%d mm, (%d+%d)/%d deg]\n", mMoveAccumulator, d_move, mMoveTarget, mTurnAccumulator, d_turn, mTurnTarget);
#endif

    // update.
    // naughty hack to stop weird ass values being introduced
    if (mTurnTarget != 0)
        mTurnAccumulator += d_turn;
    if (mMoveTarget != 0)
        mMoveAccumulator += d_move;
    
    // check if we have hit our targets.
    // have we finished turning?
    if ((mTurnTarget > 0 && mTurnAccumulator >= mTurnTarget) ||
        (mTurnTarget < 0 && mTurnAccumulator <= mTurnTarget)   )
    {
#ifdef VERY_VERBOSE_PRINTOUTS
        printf("  turn limit reached\n");
#endif
        if (mMoveAccumulator != 0)
            printf("WARNING: we somehow accumulated movement despite actually rotating (a=%d). Zeroing out.\n", mMoveAccumulator);
        zeroTargetsAndAccumulators();
    }
    // have we finished moving?
    if ((mMoveTarget > 0 && mMoveAccumulator >= mMoveTarget) ||
        (mMoveTarget < 0 && mMoveAccumulator <= mMoveTarget)   )
    {
#ifdef VERY_VERBOSE_PRINTOUTS
        printf("  move limit reached\n");
#endif
        if (mTurnAccumulator != 0)
            printf("WARNING: we somehow accumulated rotation despite actually moving (a=%d). Zeroing out.\n", mTurnAccumulator);
        zeroTargetsAndAccumulators();
    }
}


/// @brief  Stop right now, thank you very much. We need somebody with a human touch.
void Robot::zeroTargetsAndAccumulators (void)
{
    if (mMoveAccumulator != 0 || mTurnAccumulator != 0)
        mMap.addRelativeReadings(mMoveAccumulator, mTurnAccumulator);
    mMoveAccumulator = 0;
    mTurnAccumulator = 0;
    mMoveTarget = 0;
    mTurnTarget = 0;
    setSpeed(0, 0);
    mIsIdle = true;
}


void Robot::startNapping (int duration)
{
    mIsNapping = true;
    mNapDuration = duration;
    time(&mNapStarted);
}

bool Robot::nappingTimeUp (void)
{
    time_t now;
    time(&now);
    int current_nap_duration = (int) difftime(now, mNapStarted);
#ifdef VERY_VERBOSE_PRINTOUTS
    printf("    nap duration = %d / %d\n", current_nap_duration, mNapDuration);
#endif
    return (current_nap_duration >= mNapDuration);
}


void Robot::processMotorActions (Vision* vision)
{
    if (mIsIdle)
    {
        // grab the next pathing action and convert it down to motor actions
        if (mMotorActions.empty())
        {
            if (mCurrentAction.type == DIRECT_ORDER)
            {
                if (vision->checkForOccupancy())
                {
                    mMap.addTable(mMap.mCurrentNode);
                    startNapping(TARGET_NAP_DURATION);
                    mCurrentAction.type = NONE;
                    return;
                }
            }
            if (!mPathingActions.empty())
            {
                executePathingAction();
            }
        }
        // not else if because executePathingAction (called above) will populate the list.
        if (!mMotorActions.empty())
        {
            // grab the next motor action
            MotorAction next = mMotorActions.front();
            mMotorActions.pop();
            
            // load her up
            zeroTargetsAndAccumulators();
            if (next.type == ROTATION)
            {
//                targetRotation(next.action.angle);
                updateTargets(0, next.action.angle);
                if (next.action.angle > 0)
                    setSpeed(ROBOT_TURN_SPEED, -ROBOT_TURN_SPEED);  // CW
                else
                    setSpeed(-ROBOT_TURN_SPEED, ROBOT_TURN_SPEED);  // CCW
            }
            else
            {
                updateTargets(next.action.displacement, 0);
                if (next.action.displacement > 0)
                    setSpeed(ROBOT_MOVE_SPEED, ROBOT_MOVE_SPEED);
                else
                    setSpeed(-ROBOT_MOVE_SPEED, -ROBOT_MOVE_SPEED);
            }
        }
    }
}

void Robot::executePathingAction (void)
{
    mCurrentAction = mPathingActions.front();
    mPathingActions.pop();
    
    ActionPriority priority = LEVEL1;
    if (mCurrentAction.type == GREEDY_TABLE)
        priority = LEVEL2;
    
    if (mCurrentAction.first_angle != 0)
        mMotorActions.push(MotorAction(ROTATION,    mCurrentAction.first_angle,  priority));
    if (mCurrentAction.displacement != 0)
        mMotorActions.push(MotorAction(TRANSLATION, mCurrentAction.displacement, priority));
    if (mCurrentAction.final_angle != 0)
        mMotorActions.push(MotorAction(ROTATION,    mCurrentAction.final_angle,  priority));
}


/// @brief  Reroutes the the path to the final target in the pathing actions queue via only graph
///         nodes (using the closest, backward node first).
void Robot::reroutePathingActions (void)
{
    // work out where we're actually going and then clear the whole queue - we don't need any of that crap.
    Point2i  final_destination = mPathingActions.back().target;
    uint32_t final_destination_node = mMap.lookupPoint(final_destination);
    dropPathingActions();
    if (final_destination_node == UINT32_MAX)
    {
        printf("oh no\n");
        return;
    }
    
    // Get the nearest backward node.
    uint32_t nearest_back_node = mMap.getNearestBackwardNode(mMap.mGraph[mMap.mCurrentNode].p);
    
    // add the pathing action from the current position to the nearest backward node
    mPathingActions.push(generateN2NPathingAction(mMap.mCurrentNode, nearest_back_node));
    
    // Get the shortest path from that node to the target.
    std::vector<uint32_t> shortest_path = mMap.dijkstra(nearest_back_node, final_destination_node);
    uint32_t i, shortest_path_length = shortest_path.size();
    if (shortest_path_length > 0)
        for (i = 0; i < shortest_path_length-1; i++)
            mPathingActions.push(generateN2NPathingAction(shortest_path[i], shortest_path[i+1]));
}

void Robot::dropPathingActions (void)
{
    // clear the two queues out. an annoying oversight of the STL is they have no clear function so
    // this must be done manually (swap is used to ensure the memory is actually zeroed and prevent
    // the queues being little divas and hanging onto their items silently).
    std::queue<PathingAction>  emptyPathing;
    std::queue<MotorAction>    emptyMotor;
    std::swap(mPathingActions, emptyPathing);
    std::swap(mMotorActions,   emptyMotor);
    // also zero out accumulators
    zeroTargetsAndAccumulators();
    // we now having nothing to do, lovely.
    mIsIdle = true;
}


void Robot::timestep (Vision* vision, sint8_t object_avoidance, bool target_recognition, MarkerData& target_recognition_data)
{
    static  PathingAction action;
    uint8_t bumperValues[2];
    int     next_rotation_offset = 0;
    bool    action_generated = false;
    bool    has_napped = false;
    static int nap_cooldown = 0;
    
#ifdef VERY_VERBOSE_PRINTOUTS
    printf("  lets path\n");
#endif
    
    // bookkeeping
    mMap.updateWeights();
    
    // handle napping and exit straight out if we are currently napping.
    if (mIsNapping)
    {
#ifdef VERY_VERBOSE_PRINTOUTS
        printf("  napping\n");
#endif
        if (!nappingTimeUp())
            return;
        mIsNapping = false;
        has_napped = true;
        nap_cooldown = 25;
    }
    else if (nap_cooldown > 0)
    {
        nap_cooldown--;
    }
    
    // update the accumulators
    const int latestDistance = (int) getDistance();
    const int latestAngle    = (int) getAngle();
    updateAccumulators(latestDistance, latestAngle);
    
    // check bumpers. stuff touching the bumpers = immediate problem which stops for no-one.
    if (!action_generated)
    {
        getBumperValues(bumperValues);
        if ((bumperValues[0] || bumperValues[1]) && !has_napped && nap_cooldown == 0)
        {
#ifdef VERBOSE_PRINTOUTS
            printf("  EW SOMETHING ON THE BUMPERS. sleeping\n");
#endif
            startNapping(PATHING_NAP_DURATION);
            return;
        }
        else if ((bumperValues[0] || bumperValues[1]) && has_napped)
        {
#ifdef VERBOSE_PRINTOUTS
            printf("  EW SOMETHING STILL ON THE BUMPERS HAVING SLEPT.\n");
#endif
            zeroTargetsAndAccumulators();
            
            if (mCurrentAction.type == GREEDY_TABLE)
            {
#ifdef VERBOSE_PRINTOUTS
                printf("  - rerouting\n");
#endif
                reroutePathingActions();
            }
            else
            {
#ifdef VERBOSE_PRINTOUTS
                printf("  - dropping\n");
#endif
                dropPathingActions();
                
                if (bumperValues[0] && !bumperValues[1])        // just the left bumper (so rotate right).
                    next_rotation_offset = 20;
                else if (!bumperValues[0] &&  bumperValues[1])  // just the right bumper (so rotate left).
                    next_rotation_offset = -20;
                else                                            // both bumpers, spin right round, right round.
                    next_rotation_offset = 20;
                
                mPathingActions.push(generateRandomPathingAction(next_rotation_offset));
            }
            action_generated = true;
        }
    }
    
    // check object avoidance. our response to this will depend what we're currently doing.
    if (!action_generated)
    {
        if (object_avoidance && !has_napped)
        {
#ifdef VERBOSE_PRINTOUTS
            printf("  PANICKING FROM THE CAMERA (%c). sleeping\n", (object_avoidance > 1) ? 's' : 'd');
#endif
            // we've seen something freaky - wait for 1s and see if it's still there.
            startNapping(PATHING_NAP_DURATION);
            return;
        }
        else if (object_avoidance && has_napped)
        {
#ifdef VERBOSE_PRINTOUTS
            printf("  STILL PANICKING FROM THE CAMERA (%c) HAVING SLEPT\n", (object_avoidance > 1) ? 's' : 'd');
#endif
            // okay now we've had a little snooze, but there's still something freaky as shit out there.
            // TAKE EVASIVE ACTION.
            zeroTargetsAndAccumulators();
            
            if (mCurrentAction.type == GREEDY_TABLE)
            {
#ifdef VERBOSE_PRINTOUTS
                printf("  - rerouting\n");
#endif
                reroutePathingActions();
            }
            else
            {
#ifdef VERBOSE_PRINTOUTS
                printf("  - dropping\n");
#endif
                dropPathingActions();
                
                if (object_avoidance == -1)                     // just something scary on the left, turn right.
                    next_rotation_offset = 20;
                else if (object_avoidance == 1)                 // just something scary on the right, turn left.
                    next_rotation_offset = -20;
                else                                            // both sides :(
                    next_rotation_offset = 40;
                
                mPathingActions.push(generateRandomPathingAction(next_rotation_offset));
            }
            action_generated = true;
        }
    }
    
    // check marker detection, but only respond to it if we're not already doing something important.
    if (!action_generated)
    {
        if (target_recognition)
        {
#ifdef VERBOSE_PRINTOUTS
            printf("  marker seen!\n");
#endif
            if (mCurrentAction.type != GREEDY_TABLE || (mCurrentAction.type == GREEDY_TABLE && mPathingActions.size() < 1))
            {
#ifdef VERBOSE_PRINTOUTS
                printf("  - lets go!\n");
#endif
                zeroTargetsAndAccumulators();
                dropPathingActions();
                
                mPathingActions.push(generateMarkerPathingAction(target_recognition_data));
                
                action_generated = true;
            }
            else
            {
#ifdef VERBOSE_PRINTOUTS
                printf("  - nvm\n");
#endif
            }
        }
    }
    
    // if we've got through that gauntlet it means we're chilling out safetly and have nothing to do!
    // we should probably find something to do tbh...
//    if (!action_generated && mPathingActions.size() == 0 && mMotorActions.size() == 0)
    if (!action_generated && mIsIdle)
    {
#ifdef VERBOSE_PRINTOUTS
        printf("  nothing better to do, lets random it up\n");
#endif
        PathingAction action = generateRandomPathingAction(0);
#ifdef VERBOSE_PRINTOUTS
        printf("  - type = %d, target = (%d,%d) = %d deg -> %d mm -> %d deg\n", (int) action.type, action.target.x, action.target.y, action.first_angle, action.displacement, action.final_angle);
#endif
        mPathingActions.push(action);
    }
    
    // actually execute the jobs we've spent so long constructing.
#ifdef VERY_VERBOSE_PRINTOUTS
    printf("  executing motor actions\n");
#endif
    processMotorActions(vision);
}


PathingType Robot::generateRandomPathingType (void)
{
    float map_size    = (float) mMap.mGraph.size();
    float table_count = (float) mMap.mTableNodes.size();
    
    float P_exploit = (table_count < 1) ? 0 : (((float) mMap.tableWeightsSum()) / (table_count * 100.0)) * (1 - (1.0 / table_count+1));
    float P_movement_node = 0.1 * (1 - (1.0 / map_size+1));
    
//    float P_movement_table = (1 - (1.0 / (mMap.mGraph.size()+1))) * 0.5;
    float rand1 = ((float) rand()) / ((float) RAND_MAX);
    
    if (rand1 > P_exploit)
    {
        float rand2 = ((float) rand()) / ((float) RAND_MAX);
        if (rand2 < P_movement_node)
            return GREEDY_NODE;
        else
            return RANDOM;
    }
    else
    {
        return GREEDY_TABLE;
    }
}


PathingAction Robot::generateRandomPathingAction (int rotation_mean)
{
    Point2i currentPosition = mMap.mGraph[mMap.mCurrentNode].p;
    PathingAction action;
    PathingType pathingType = generateRandomPathingType();
    
    if (pathingType == RANDOM)
    {
        // generate a random angle.
        int angle = (int) randNormallyDistributed(rotation_mean, PATHING_ANGULAR_STD_DEV);
        // work out the position that angle would take you to.
        Point2i newPosition(currentPosition, PATHING_MAX_MOVE_DISTANCE, angle);
        // load up the target.
        action.type = pathingType;
        action.target = newPosition;
        action.first_angle = angle;
        action.displacement = PATHING_MAX_MOVE_DISTANCE;
        action.final_angle = 0;
    }
    else if (pathingType == GREEDY_TABLE)
    {
        // generate a random table weighted by the tables' weights.
        int random_table = rand() % mMap.tableWeightsSum();
        // work out who owns that random 'weight'.
        int accumulator = 0, table_index = -1;
        do {
            accumulator += mMap.mTableNodes[++table_index].second;
        } while (accumulator < random_table);
        // load up the target with that table
        action = generateN2NPathingAction(mMap.mGraph[mMap.mCurrentNode].p, mMap.mGraph[mMap.mTableNodes[table_index].first].p);
        action.type = pathingType;
/*        action.target = mMap.mGraph[mMap.mTableNodes[table_index].first].p;
        // calculate the angles
        float dx = action.target.x - currentPosition.x;
        float dy = action.target.y - currentPosition.y;
        action.first_angle = ((int) RADTODEG(atan(dx/dy))) - mMap.mCurrentOrientation;
        action.displacement = (int) euclidean_distance(action.target, currentPosition);
        action.final_angle = 0;*/
    }
    else
    {
        // generate a random node.
        int random_node = rand() % mMap.mGraph.size();
        // load up the target.
        action = generateN2NPathingAction(mMap.mGraph[mMap.mCurrentNode].p, mMap.mGraph[random_node].p);
        action.type = pathingType;
/*        action.target = mMap.mGraph[random_node].p;
        // calculate the angles
        float dx = action.target.x - currentPosition.x;
        float dy = action.target.y - currentPosition.y;
        action.first_angle  = ((int) RADTODEG(atan(dx/dy))) - mMap.mCurrentOrientation;
        action.displacement = (int) euclidean_distance(action.target, currentPosition);
        action.final_angle = 0;*/
    }
    
    return action;
}


PathingAction Robot::generateN2NPathingAction (const uint32_t start, const uint32_t end)
{
    return generateN2NPathingAction(mMap.mGraph[start].p, mMap.mGraph[end].p);
}


PathingAction Robot::generateN2NPathingAction (const Point2i& start, const Point2i& end)
{
    float currentOrientation = (float) mMap.mCurrentOrientation;
    Vector2 S(sin(DEGTORAD(currentOrientation)), cos(DEGTORAD(currentOrientation)), VECTOR_AUTONORMALISED);
    Vector2 T(((float) end.x - start.x),         ((float) end.y - start.y),         VECTOR_AUTONORMALISED);
    
    float angle = S.angleTo(T);
    float dist  = euclidean_distance(start, end);
    
    PathingAction action;
    action.target = end;
    action.first_angle  = (int) angle;
    action.displacement = (int) dist;
    action.final_angle  = 0;
    
    return action;
}


/// @brief  TODO.
PathingAction Robot::generateMarkerPathingAction (MarkerData& marker_data)
{
    PathingAction action;
    Point2i currentPosition = mMap.mGraph[mMap.mCurrentNode].p;
    
    // first calculate the co-ordinates of the target point relative to the marker.
    float z_p = PATHING_MARKER_STOPPING_DISTANCE * cos(-DEGTORAD(marker_data.orientation));
    float x_p = PATHING_MARKER_STOPPING_DISTANCE * sin(-DEGTORAD(marker_data.orientation));
    
    // then calculate the x and z displacements required to reach that point from the current point (0,0)
    float z_pp = marker_data.position.z - z_p;
    float x_pp = marker_data.position.x - x_p;
    
    // and finally the angle and displacement actually needed.
    float h_d = sqrt((x_pp * x_pp) + (z_pp * z_pp));
    float theta = RADTODEG(atan(x_pp / z_pp));
    
    // print them
    printf("h_d = %.1f, theta = %.1f\n", h_d, theta);
    
    // now save these to the PathingAction
    action.type = DIRECT_ORDER;
    action.target = Point2i(currentPosition.x + x_pp, currentPosition.y + z_pp);
    action.first_angle  = (int) theta;
    action.displacement = (int) h_d;
    action.final_angle  = (int) ((-marker_data.orientation) - theta);
    
    return action;
}

/*
/// @brief  converts a path to a load of pathing actions
std::vector<PathingAction> Robot::generateShortestPathingActions (uint32_t target_node)
{
    std::vector<PathingAction> r;
    std::vector<uint32_t> shortest_path = mMap.dijkstra(mMap.mCurrentNode, target_node);
    uint32_t i, shortest_path_length = shortest_path.size();
    
    if (shortest_path_length > 0)
        for (i = 0; i < shortest_path_length-1; i++)
            r.push_back(generateN2NPathingAction(shortest_path[i], shortest_path[i+1]));
    
    return r;
}


/// @brief  Takes the supplied PathingAction and turns it into a bunch of MotorActions which are
///         then added to the action queue to be done when it finishes the current stuff in there.
void Robot::executePathingAction (PathingAction& action)
{
    ActionPriority priority = LEVEL1;
    if (action.type == GREEDY_TABLE)
        priority = LEVEL2;
    
    if (action.first_angle != 0)
        mMotorActions.push_back(MotorAction(ROTATION,     action.first_angle,  priority));
    if (action.displacement != 0)
        mMotorActions.push_back(MotorAction(DISPLACEMENT, action.displacement, priority));
    if (action.final_angle != 0)
        mMotorActions.push_back(MotorAction(ROTATION,     action.final_angle,  priority));
}


void Robot::executePathingActions (std::vector<PathingAction>& actions)
{
    for (uint32_t i = 0; i < actions.size(); i++)
        executePathingAction(actions[i]);
}*/
