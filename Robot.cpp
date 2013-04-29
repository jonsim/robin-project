#include "Robot.h"


/// @brief  Constructor. Opens and initialises the serial connection before turning on the Robot. It
///         starts in PASSIVE mode. See setMode() for further information.
Robot::Robot (void) : mCurrentMode(OFF), mMoveAccumulator(0), mMoveTarget(0), mTurnAccumulator(0), mTurnTarget(0), mIsInterruptable(true), mIsIdle(true)
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
    
    mSI->writeBytes(command, sizeof(command));
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
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 19u};
    uint8_t response[2];
    
    mSI->writeBytes(command, sizeof(command));
    mSI->readBytes(response, sizeof(response));
    
    return make_sint16_t(response[0], response[1]);
}


/// @brief  Gets the angle in degrees that the Robot has turned since the angle was last requested.
///         Counter-clockwise angles are positive and clockwise angles are negative. If the value is
///         not polled frequently enough, it is capped at its min/max. 
/// @return The angle turned. Range: -32768 - 32767.
const sint16_t Robot::getAngle (void) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 20u};
    uint8_t response[2];
    
    mSI->writeBytes(command, sizeof(command));
    mSI->readBytes(response, sizeof(response));
    
    return make_sint16_t(response[0], response[1]);
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
}


/// @brief  Sets the speed of the Robot's wheels.
/// @param  lVel  The velocity of the left  wheel in mm/s. Range = -500 - 500.
/// @param  rVel  The velocity of the right wheel in mm/s. Range = -500 - 500.
void Robot::setSpeed (const sint16_t lVel, const sint16_t rVel)
{
    static sint16_t curr_lVel=0, curr_rVel=0;
    CHECK_ROBOTMODE(SAFE);
    
    if (lVel != curr_lVel || rVel != curr_rVel)
    {
        curr_lVel = lVel;
        curr_rVel = rVel;
        uint8_t command[5] = {145u,
                              (uint8_t) (lVel >> 8), (uint8_t) (lVel & 0xFF),
                              (uint8_t) (rVel >> 8), (uint8_t) (rVel & 0xFF) };
        mSI->writeBytes(command, sizeof(command));
    }
}


/// @brief  Rotates the Robot a given angle. THIS RESETS THE ANGLE ACCUMULATOR READ BY getAngle().
/// @param  dc  The required rotation in degrees clockwise. A negative value represents an 
///             anti-clockwise rotation.
void Robot::targetRotation (const sint16_t dc)
{
    CHECK_ROBOTMODE(SAFE);
    uint16_t ndc = -dc;
    
    uint8_t command_header[2] = {152u, 8u};
    uint8_t command_rotate[5] = {137u, 0u, 0u, (dc < 0) ? 0u : 255u, (dc < 0) ? 1u : 255u};
    uint8_t command_wait[3]   = {157u, (uint8_t) (ndc >> 8), (uint8_t) (ndc & 0xFF)};
    
    mSI->writeBytes(command_header, sizeof(command_header));
    mSI->writeBytes(command_rotate, sizeof(command_rotate));
    mSI->writeBytes(command_wait,   sizeof(command_wait  ));
    mSI->writeByte(153u);
    
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
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {137u, demo_number};
    
    mSI->writeBytes(command, sizeof(command));
}


/// @brief  Halts the currently running demo. Does nothing if there is not a demo currently running.
void Robot::stopDemo (void)
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {137u, 255u};
    
    mSI->writeBytes(command, sizeof(command));
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


/// @brief  Updates the Robot's internal map with the latest actuator readings. It is strongly
///         recommended to call this every time a new movement instruction is issued.
void Robot::updateMap (void)
{
    const sint16_t latestDistance = getDistance();
    const sint16_t latestAngle = getAngle();
    mMap.addRelativeReadings(latestDistance, latestAngle);
}


void Robot::updateTargets (int new_move, int new_turn)
{
    mMoveTarget = new_move;
    mTurnTarget = new_turn;
    mIsIdle = false;
}


void Robot::updateAccumulators (int d_move, int d_turn)
{
    // update.
    mMoveAccumulator += d_move;
    mTurnAccumulator += d_turn;
    
    // check if we have hit our targets.
    // have we finished turning?
    if ((mTurnTarget > 0 && mTurnAccumulator >= mTurnTarget) ||
        (mTurnTarget < 0 && mTurnAccumulator <= mTurnTarget)   )
    {
        mMap.addRelativeReadings(mMoveAccumulator, mTurnAccumulator);
        mTurnAccumulator = 0;
        mTurnTarget = 0;
        if (mMoveTarget < mMoveAccumulator)
            setSpeed(-ROBOT_MOVE_SPEED, -ROBOT_MOVE_SPEED);
        else if (mMoveTarget > mMoveAccumulator)
            setSpeed(ROBOT_MOVE_SPEED, ROBOT_MOVE_SPEED);
        else
            setSpeed(0, 0);
    }
    // have we finished moving?
    if ((mMoveTarget > 0 && mMoveAccumulator >= mMoveTarget) ||
        (mMoveTarget < 0 && mMoveAccumulator <= mMoveTarget)   )
    {
        mMap.addRelativeReadings(mMoveAccumulator, mTurnAccumulator);
        mMoveAccumulator = 0;
        mMoveTarget = 0;
        if (mTurnTarget < mTurnAccumulator)
            setSpeed(ROBOT_TURN_SPEED, -ROBOT_TURN_SPEED);
        else if (mTurnTarget > mTurnAccumulator)
            setSpeed(-ROBOT_TURN_SPEED, ROBOT_TURN_SPEED);
        else
            setSpeed(0, 0);
    }
    // have we finished what we set out to do?
    if (mMoveTarget == 0 && mTurnTarget == 0)
        mIsIdle = true;
}


void Robot::zeroTargetsAndAccumulators (void)
{
    if (!(mMoveAccumulator == 0 && mTurnAccumulator == 0))
        mMap.addRelativeReadings(mMoveAccumulator, mTurnAccumulator);
    mMoveAccumulator = 0;
    mTurnAccumulator = 0;
    mMoveTarget = 0;
    mTurnTarget = 0;
    setSpeed(0, 0);
    mIsIdle = true;
}


void Robot::timestep (sint8_t object_avoidance, bool target_recognition, MarkerData& target_recognition_data)
{
    static PathingAction action;
    uint8_t bumperValues[2];
    int     next_rotation_offset = 0;
    bool    okay_to_proceed = true;
    bool has_napped = false;
    
    // handle napping and exit straight out if we are currently napping.
    if (mIsNapping)
    {
        time_t now;
        time(&now);
        int nap_duration = (int) difftime(now, mNapStarted);
        if (nap_duration < PATHING_NAP_DURATION)
            return;
        mIsNapping = false;
        has_napped = true;
    }
    
    // update  the values
    const sint16_t latestDistance = getDistance();
    const sint16_t latestAngle = getAngle();
    updateAccumulators(latestDistance, latestAngle);
    
    // check bumpers. stuff touching the bumpers = immediate problem which stops for no-one.
    if (okay_to_proceed)
    {
        getBumperValues(bumperValues);
        if (bumperValues[0] || bumperValues[1])
        {
            zeroTargetsAndAccumulators();
            okay_to_proceed = false;
        }
        if (      bumperValues[0] && !bumperValues[1])  // just the left bumper (so rotate right).
        {
            next_rotation_offset = 90;
            action = generateRandomPathingAction(next_rotation_offset);
        }
        else if (!bumperValues[0] &&  bumperValues[1])  // just the right bumper (so rotate left).
        {
            next_rotation_offset = -90;
            action = generateRandomPathingAction(next_rotation_offset);
        }
    }
    
    // check object avoidance. our response to this will depend what we're currently doing.
    if (okay_to_proceed)
    {
        if (object_avoidance != 0 && !has_napped)
        {
            // we've seen something freaky
            // wait for 1s and see if it's still there.
            mIsNapping = true;
            time(&mNapStarted);
            return;
        }
        else if (object_avoidance != 0 && has_napped)
        {
            // okay now we've had a little snooze, but there's still something freaky as shit out there.
            // TAKE EVASIVE ACTION.
            zeroTargetsAndAccumulators();
            if (object_avoidance == -1)         // just something scary on the left, turn right.
                next_rotation_offset = 90;
            if (object_avoidance == 1)          // just something scary on the right, turn left.
                next_rotation_offset = -90;
            action = generateRandomPathingAction(next_rotation_offset);
            okay_to_proceed = false;
        }
    }
    
    // check marker detection, but only respond to it if we're not already doing something important.
    if (okay_to_proceed)
    {
        if (target_recognition && mIsInterruptable)
        {
            action = generateMarkerPathingAction(target_recognition_data);
            okay_to_proceed = false;
        }
    }
    
    // if we've got through that gauntlet it means we're chilling out safetly and have nothing to do!
    // we should probably find something to do tbh...
    action = generateRandomPathingAction(next_rotation_offset);
}


PathingType Robot::generatePathingType (void)
{
    float table_weights_sum = (float) mMap.tableWeightsSum();
    float table_count       = (float) mMap.mTableNodes.size();
    float P_movement_random = 1.0 - (1.0 / table_count);
    float P_movement_table  = table_weights_sum / (table_count * 100.0);
    float rand1 = ((float) rand()) / ((float) RAND_MAX);
    float rand2 = ((float) rand()) / ((float) RAND_MAX);
    
    if (rand1 < P_movement_random)
        return RANDOM;
    else if (rand2 < P_movement_table)
        return GREEDY_TABLE;
    return GREEDY_NODE;
}


PathingAction Robot::generateRandomPathingAction (int rotation_mean)
{
    PathingAction action;
    action.type = generatePathingType();
    Point2i currentPosition = mMap.mGraph[mMap.mCurrentNode].p;
    
    if (action.type == RANDOM)
    {
        // generate a random angle.
        int angle = (int) randNormallyDistributed(rotation_mean, PATHING_ANGULAR_STD_DEV);
        // work out the position that angle would take you to.
        Point2i newPosition(currentPosition, PATHING_MAX_MOVE_DISTANCE, angle);
        // load up the target.
        action.target = newPosition;
        action.first_angle = angle;
        action.displacement = PATHING_MAX_MOVE_DISTANCE;
        action.final_angle = 0;
    }
    else if (action.type == GREEDY_TABLE)
    {
        // generate a random table weighted by the tables' weights.
        int random_table = rand() % mMap.tableWeightsSum();
        // work out who owns that random 'weight'.
        int accumulator = 0, table_index = -1;
        while (accumulator <= random_table);
            accumulator += mMap.mTableNodes[++table_index].second;
        // load up the target with that table
        action.target = mMap.mGraph[mMap.mTableNodes[table_index].first].p;
        // calculate the angles
        float dx = action.target.x - currentPosition.x;
        float dy = action.target.y - currentPosition.y;
        action.first_angle = ((int) RADTODEG(atan(dx/dy))) - mMap.mCurrentOrientation;
        action.displacement = (int) euclidean_distance(action.target, currentPosition);
        action.final_angle = 0;
    }
    else
    {
        // generate a random node.
        int random_node = rand() % mMap.mGraph.size();
        // load up the target.
        action.target = mMap.mGraph[random_node].p;
        // calculate the angles
        float dx = action.target.x - currentPosition.x;
        float dy = action.target.y - currentPosition.y;
        action.first_angle  = ((int) RADTODEG(atan(dx/dy))) - mMap.mCurrentOrientation;
        action.displacement = (int) euclidean_distance(action.target, currentPosition);
        action.final_angle = 0;
    }
    
    return action;
}


/// @brief  Uses the Box-Muller transform to generate a single normally distributed random number.
///         NB: This is not efficient. If I find myself calling it a lot consider using the polar
///             form of the transform or potentially even the Ziggurat algorithm.
///         NB: rand() must be seeded when this function is called. If in doubt, srand!
/// @param  mu      The mean of the distribution.
/// @param  sigma   The standard deviation of the distribution.
/// @return         A random number normally distributed according to the supplied parameters.
float Robot::randNormallyDistributed (float mu, float sigma)
{
    float U1 = ((float) rand()) / ((float) RAND_MAX);
    float U2 = ((float) rand()) / ((float) RAND_MAX);
    float Z0 = sqrt(-2 * log(U1)) * cos(2 * PI * U2);
    return (Z0 * sigma) + mu;
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

void Robot::executePathingAction (PathingAction& action)
{
    if (mIsIdle)
    {
        mTurnAccumulator = action.first_angle;
        mMoveAccumulator = action.displacement;
        mIsIdle = false;
    }
}
