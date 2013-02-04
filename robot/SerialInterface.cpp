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


/// @brief      Writes a single byte to the Serial Port.
/// @param b    The byte to write.
void SerialInterface::writeByte (const uint8_t b)
{
    int retVal = write(mSerialPort, &b, 1);
    CHECK_RETURN(retVal, "SerialInterface::writeByte");
}


/// @brief      Writes an array of bytes to the Serial Port.
/// @param bs   The bytes to write.
/// @param n    The number of bytes to write. ENSURE THIS IS NOT MORE THAN THE LENGTH OF THE ARRAY.
void SerialInterface::writeBytes (const uint8_t* bs, const int n)
{
    int retVal = write(mSerialPort, bs, n);
    CHECK_RETURN(retVal, "SerialInterface::writeBytes");
}


/// @brief      Reads a single byte from the Serial Port.
///             If INTERFACE_BLOCKING_READ is 1 it will wait until a byte is available to be read.
/// @return     The byte read.
uint8_t SerialInterface::readByte (void)
{
    uint8_t b;
    int retVal = read(mSerialPort, &b, 1);
    CHECK_RETURN(retVal, "SerialInterface::readByte");
    return b;
}

/// @brief      Reads a number of bytes from the Serial Port.
///             If INTERFACE_BLOCKING_READ is 1, this will read the requested number of bytes into
///             the array, otherwise it will read the requested number of bytes or until the input
///             buffer is empty, whichever is sooner.
/// @param bs   The array to read into.
/// @param n    The number of bytes to read. ENSURE THIS IS NOT MORE THAN THE LENGTH OF THE ARRAY.
void SerialInterface::readBytes (uint8_t* bs, const int n)
{
    int retVal = read(mSerialPort, bs, n);
    CHECK_RETURN(retVal, "SerialInterface::readBytes");
}
