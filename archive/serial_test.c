#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <termios.h>
#include <fcntl.h>

// DEFINES
#define INTERFACE_NAME          "/dev/ttyUSB0"
#define INTERFACE_SPEED         B57600
#define INTERFACE_DATA_BITS     8
#define INTERFACE_PARITY        0
#define INTERFACE_FLOW_CONTROL  0
#define INTERFACE_BLOCKING_READ 1    // Sets whether or not reads block or just return up to the number of bits in the buffer. Can be either 0 or 1.

// MACROS
// Check standard function return values (error if < 0) and throw an error if necessary
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }
// Check annoying non-standard return values (error if != 0) and throw an error if necessary
#define CHECK_RETURN_NEQ(r, what)   \
    if (r != 0)                     \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }



int setupSerialPort (void)
{
    int serialPort;
    int retVal;
    
    // First open the interface and check it has worked.
    serialPort = open(INTERFACE_NAME, O_RDWR | O_NOCTTY | O_SYNC);
    CHECK_RETURN(serialPort, "opening the serial interface.");
    
    // Next set the interface's parameters.
    // Create the attribute container and load it with the current values.
    struct termios serialAttributes;
    memset(&serialAttributes, 0, sizeof(serialAttributes));
    retVal = tcgetattr(serialPort, &serialAttributes);
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
    retVal = tcsetattr(serialPort, TCSANOW, &serialAttributes);
    CHECK_RETURN_NEQ(retVal, "tcsetattr");
    
    // Return.
    return serialPort;
}


int main (void)
{
    int retVal;
    char buffer[50];
    int serialPort;
    
    serialPort = setupSerialPort();

    write(serialPort, "hello!\n", 7);           // send 7 character greeting
    retVal = read(serialPort, buffer, sizeof(buffer));  // read up to 100 characters if ready to read
    printf("%s\n", buffer);
    
    return 0;
}
