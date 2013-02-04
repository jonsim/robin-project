#include "SerialInterface.h"

/// @brief  Initialises the serial port according to the INTERFACE_* definitions contained within
///         SerialInterface.h. NB: A couple of the definitions aren't implemented yet... shhh!
void SerialInterface::start (void)
{
    int retVal;
    
    // First open the interface and check it has worked.
    mSerialPort = open(INTERFACE_NAME, O_RDWR | O_NOCTTY | O_SYNC);
    CHECK_RETURN(mSerialPort, "Opening the serial interface.");
    
    // Next set the interface's parameters.
    // Create the attribute container and load it with the current values.
    struct termios serialAttributes;
    memset(&serialAttributes, 0, sizeof(serialAttributes));
    retVal = tcgetattr(mSerialPort, &serialAttributes);
    CHECK_RETURN_NEQ(retVal, "tcgetattr");
    
    // Ammend the values as appropriate.
    cfsetospeed (&serialAttributes, INTERFACE_SPEED);       // speed out
    cfsetispeed (&serialAttributes, INTERFACE_SPEED);       // speed in
    serialAttributes.c_cflag = (serialAttributes.c_cflag & ~CSIZE) | CS8;   // 8-bit chars
    serialAttributes.c_iflag &= ~IGNBRK;                    // ignore break signal
    serialAttributes.c_lflag = 0;                           // no signaling chars, no echo,
    serialAttributes.c_oflag = 0;                           // no remapping, no delays
    serialAttributes.c_cc[VMIN]  = INTERFACE_BLOCKING_READ; // set read blocking
    serialAttributes.c_cc[VTIME] = 5;                       // 0.5 seconds read timeout
    serialAttributes.c_iflag &= ~(IXON | IXOFF | IXANY);    // shut off xon/xoff ctrl
    serialAttributes.c_cflag |= (CLOCAL | CREAD);           // ignore modem controls,
    serialAttributes.c_cflag &= ~(PARENB | PARODD);         // shut off parity
    serialAttributes.c_cflag |= INTERFACE_PARITY;           // set parity
    serialAttributes.c_cflag &= ~CSTOPB;                    // set stop bits
    serialAttributes.c_cflag &= ~CRTSCTS;                   // set something else

    // Apply the attributes.
    retVal = tcsetattr(mSerialPort, TCSANOW, &serialAttributes);
    CHECK_RETURN_NEQ(retVal, "tcsetattr");
}


/// @brief      Shorthand for writeTo(str, strlen(str))
void SerialInterface::writeTo (const char* str)
{
    int retVal = write(mSerialPort, str, strlen(str));
    CHECK_RETURN(retVal, "SerialInterface::writeTo");
}


/// @brief      Writes a given number of bytes from the supplied character array to the Serial Port.
/// @param str  The character array to write.
/// @param bytes The number of bytes from the character array to write (ENSURE THIS IS NOT MORE THAN
///              THE LENGTH OF THE ARRAY).
void SerialInterface::writeTo (const char* str, const int bytes)
{
    int retVal = write(mSerialPort, str, bytes);
    CHECK_RETURN(retVal, "SerialInterface::writeTo");
}


/// @brief      Shorthand for readFrom(str, sizeof(str))
void SerialInterface::readFrom (char* str)
{
    int retVal = read(mSerialPort, str, sizeof(str));
    CHECK_RETURN(retVal, "SerialInterface::readFrom");
}


/// @brief      Reads from the Serial Port. If INTERFACE_BLOCKING_READ is 1, this will read the
///             requested number of bytes into the array (ENSURE THIS IS NOT MORE THAN THE LENGTH OF
///             THE ARRAY), otherwise it will the requested number of bytes or until the input
///             buffer is empty, whichever is sooner.
/// @param str  The character array to read into.
/// @param bytes The number of bytes to read.
void SerialInterface::readFrom (char* str, const int bytes)
{
    int retVal = read(mSerialPort, str, bytes);
    CHECK_RETURN(retVal, "SerialInterface::readFrom");
}
