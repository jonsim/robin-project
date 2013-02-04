#include "Robot.h"

Robot::Robot (void)
{
    mSI.start();
    mSI.writeByte(128u);
}



const sint16_t Robot::getDistance (void) const
{
    return -10;
}

const sint16_t Robot::getAngle (void) const
{
    return 10;
}

void Robot::setMode (const RobotMode rm)
{
    switch (rm)
    {
        case PASSIVE:   mSI.writeByte(PASSIVE); break;
        case SAFE:      mSI.writeByte(SAFE);    break;
        case FULL:      mSI.writeByte(FULL);    break;
    }
}

void Robot::setSpeed (const sint16_t lVel, const sint16_t rVel)
{
    uint8_t cmd[5] = {145u,
                      (uint8_t) (lVel >> 8),
                      (uint8_t) (lVel & 0xFF),
                      (uint8_t) (rVel >> 8),
                      (uint8_t) (rVel & 0xFF)};
    printf("setting speed: %X %X %X %X\n", (uint8_t) (lVel >> 8), (uint8_t) (lVel & 0xFF), (uint8_t) (rVel >> 8), (uint8_t) (rVel & 0xFF));
    mSI.writeBytes(cmd, sizeof(cmd));
}

void Robot::setLEDs (const bool playLED, const bool advanceLED)
{
    uint8_t led_bits = 0;
    if (playLED)
        led_bits |= 2;
    if (advanceLED)
        led_bits |= 8;
    uint8_t cmd[4] = {139, led_bits, 0, 255};
    mSI.writeBytes(cmd, sizeof(cmd));
}

void Robot::printCharging (void) const
{
}

void Robot::printStatus (void) const
{
}




/// @brief  Unit testing.
int main (void)
{
    Robot fred;
    fred.setMode(FULL);
    fred.setSpeed(50, 50);
    fred.setLEDs(true, true);
    sleep(1);
    fred.setSpeed(0, 0);
    fred.setLEDs(false, false);
    
    return 0;
}
