#include "Robot.h"

Robot::Robot (void)
{
    // Nothing
}




/// @brief  Unit testing.
int main (void)
{
    char buffer[50];
    SerialInterface si;
    
    si.start();
    si.readFrom(buffer);
    
    printf("%s\n", buffer);
    
    return 0;
}
