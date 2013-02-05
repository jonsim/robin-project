#include "Robot.h"


/// @brief  Constructor. Opens and initialises the serial connection before turning on the Robot. It
///         starts in PASSIVE mode. See setMode() for further information.
Robot::Robot (void) : mCurrentMode(OFF)
{
    mSI = new SerialInterface();
    mSI->start();
    mSI->writeByte(128u);
    mCurrentMode = PASSIVE;
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
    
    for (uint8_t i = 0; i < 4; i++)
    {
        mSI->writeBytes(command[i], sizeof(command[i]));
        r[i] = mSI->readByte();
    }
}


/// @brief  Gets the boolean values of the front bumper (stored as uint8_t for efficiency).
/// @param  The array into which the boolean bumper values will be stored. This must be 2 elements
///         long (or more). The values are stored as: [0] = Left, [1] = Right.
void Robot::getBumperValues (uint8_t* r) const
{
    CHECK_ROBOTMODE(PASSIVE);
    uint8_t command[2] = {142u, 7u};
    uint8_t response;
    
    mSI->writeBytes(command, sizeof(command));
    response = mSI->readByte();
    
    r[0] = response & 2;
    r[1] = response & 1;
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
    
    r[0] = response & 8;
    r[1] = response & 4;
    r[2] = response & 16;
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
}


/// @brief  Sets the speed of the Robot's wheels.
/// @param  lVel  The velocity of the left  wheel in mm/s. Range = -500 - 500.
/// @param  rVel  The velocity of the right wheel in mm/s. Range = -500 - 500.
void Robot::setSpeed (const sint16_t lVel, const sint16_t rVel)
{
    CHECK_ROBOTMODE(SAFE);
    uint8_t command[5] = {145u,
                          (uint8_t) (lVel >> 8), (uint8_t) (lVel & 0xFF),
                          (uint8_t) (rVel >> 8), (uint8_t) (rVel & 0xFF) };
    mSI->writeBytes(command, sizeof(command));
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




/// @brief  Unit testing.
int main (void)
{
    Robot fred;
    /*
    fred.setMode(FULL);
    fred.setSpeed(50, 50);
    fred.setLEDs(true, true);
    sleep(1);
    fred.setSpeed(0, 0);
    fred.setLEDs(false, false);*/
    
    fred.printChargingStatus();
    fred.printBatteryStatus();
    
    return 0;
}
