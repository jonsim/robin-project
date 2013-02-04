/**
 * @file        SerialInterface.h
 * @brief       Functions to control how the robot communicates via serial.
 */
#ifndef SERIALINTERFACE_H
#define SERIALINTERFACE_H

/*-------------------- INCLUDES --------------------*/
#include "Common.h"
#include <termios.h>
#include <fcntl.h>


/*-------------------- DEFINES  --------------------*/
#define INTERFACE_NAME          "/dev/ttyUSB0"
#define INTERFACE_SPEED         B57600
#define INTERFACE_DATA_BITS     8
#define INTERFACE_PARITY        0
#define INTERFACE_FLOW_CONTROL  0
#define INTERFACE_BLOCKING_READ 1    // Sets whether or not reads block or just return up to the number of bits in the buffer. Can be either 0 or 1.


/*---------------- CLASS DEFINITION ----------------*/
class SerialInterface
{
public:
    SerialInterface (void) {}
    ~SerialInterface (void) {}
    
    void start (void);
    void writeByte  (const uint8_t  b);
    void writeBytes (const uint8_t* bs, const int n);
    uint8_t readByte  (void);
    void    readBytes (uint8_t* bs, const int n);

private:
    int mSerialPort;
};

#endif
