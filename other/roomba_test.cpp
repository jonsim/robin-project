#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>

#define TEST_1
//#define TEST_2
// Check standard function return values (error if < 0) and throw an error if necessary
#define CHECK_RETURN(r, what)       \
    if (r < 0)                      \
    {                               \
        perror(what);               \
        exit(EXIT_FAILURE);         \
    }


int mSerialPort;

/// @brief  Initialises the serial port according to the INTERFACE_* definitions contained within
///         SerialInterface.h. NB: A couple of the definitions aren't implemented yet... shhh!
void setup_serial (void)
{
    int retVal;
    
    // First open the interface and check it has worked.
    mSerialPort = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    CHECK_RETURN(mSerialPort, "ERROR: Could not open the serial interface");
    
    // Next set the interface's parameters.
    // Create the attribute container and load it with the current values.
    struct termios serialAttributes;
    memset(&serialAttributes, 0, sizeof(serialAttributes));
    retVal = tcgetattr(mSerialPort, &serialAttributes);
    if (retVal != 0)
    {
        perror("ERROR: tcgetattr");
        exit(EXIT_FAILURE);
    }
    
    // Ammend the values as appropriate.
    cfsetospeed (&serialAttributes, B57600);       // speed out
    cfsetispeed (&serialAttributes, B57600);       // speed in
    serialAttributes.c_cflag = (serialAttributes.c_cflag & ~CSIZE) | CS8;   // 8-bit chars
    serialAttributes.c_iflag &= ~IGNBRK;                    // ignore break signal
    serialAttributes.c_lflag = 0;                           // no signaling chars, no echo,
    serialAttributes.c_oflag = 0;                           // no remapping, no delays
    serialAttributes.c_cc[VMIN]  = 1;                       // set read blocking
    serialAttributes.c_cc[VTIME] = 5;                       // 0.5 seconds read timeout
    serialAttributes.c_iflag &= ~(IXON | IXOFF | IXANY);    // shut off xon/xoff ctrl
    serialAttributes.c_cflag |= (CLOCAL | CREAD);           // ignore modem controls,
    serialAttributes.c_cflag &= ~(PARENB | PARODD);         // shut off parity
    serialAttributes.c_cflag |= 0;                          // set parity
    serialAttributes.c_cflag &= ~CSTOPB;                    // set stop bits
    serialAttributes.c_cflag &= ~CRTSCTS;                   // set endings

    // Apply the attributes.
    retVal = tcsetattr(mSerialPort, TCSANOW, &serialAttributes);
    if (retVal != 0)
    {
        perror("ERROR: tcsetattr");
        exit(EXIT_FAILURE);
    }
}


int get_distance (void)
{
    int retVal;
    unsigned char command[2];
    unsigned char response[2];
    // read off the distance travelled (response[0] is the high byte).
    command[0] = 142u;
    command[1] =  19u;
    usleep(5000);
    retVal = write(mSerialPort, command, 2);
    CHECK_RETURN(retVal, "ERROR: write_distance");
    retVal = read(mSerialPort, response, 2);
    CHECK_RETURN(retVal, "ERROR: read_distance");
    return (response[1] + ((response[0] & 0x7f) << 8) - ((response[0] & 0x80) << 8));
}

int get_angle (void)
{
    int retVal;
    unsigned char command[2];
    unsigned char response[2];
    // read off the angle travelled (response[0] is the high byte).
    command[0] = 142u;
    command[1] =  20u;
    usleep(5000);
    retVal = write(mSerialPort, command, 2);
    CHECK_RETURN(retVal, "ERROR: write_angle");
    retVal = read(mSerialPort, response, 2);
    CHECK_RETURN(retVal, "ERROR: read_angle");
    return (response[1] + ((response[0] & 0x7f) << 8) - ((response[0] & 0x80) << 8));
}



int main (void)
{
    int retVal;
    unsigned char command[20];
    unsigned char response[5];
    signed short int distance_travelled, angle_travelled;
    
    // establish the connection to the roomba.
    setup_serial();
    
    // wait for the serial to properly start
    usleep(500000);
    printf("Serial connection established\n");
    
    // start the roomba
    command[0] = 128u;
    retVal = write(mSerialPort, command, 1);
    CHECK_RETURN(retVal, "ERROR: write0");
    // set the roomba into safe mode.
    command[0] = 131u;
    retVal = write(mSerialPort, command, 1);
    CHECK_RETURN(retVal, "ERROR: write1");
    
    // wait for the mode update to take effect
    usleep(500000);
    printf("Roomba mode set\n");
    
    // ---------- TEST 1 ----------
#ifdef TEST_1
    // set both wheels to drive
    command[0] = 145u;
    command[1] =   0u;
    command[2] = 100u;
    command[3] =   0u;
    command[4] = 100u;
    retVal = write(mSerialPort, command, 5);
    CHECK_RETURN(retVal, "ERROR: write2");
    printf("Driving\n");
    
    // wait
    //sleep(3);
    
    for (int i = 0; i < 30; i++)
    {
        usleep(30000); // 100 ms
        // read off the distance travelled (response[0] is the high byte).
        distance_travelled = get_distance();
        // read off the angle travelled (response[0] is the high byte).
        angle_travelled = get_angle();
        printf("Distance travelled = %d, angle travelled = %d\n", distance_travelled, angle_travelled);
    }
    
    // stop both wheels
    command[0] = 145u;
    command[1] =   0u;
    command[2] =   0u;
    command[3] =   0u;
    command[4] =   0u;
    retVal = write(mSerialPort, command, 5);
    CHECK_RETURN(retVal, "ERROR: write3");
    printf("Stopping\n");
    
//    usleep(10000);
    // read off the distance travelled (response[0] is the high byte).
    distance_travelled = get_distance();
    // read off the angle travelled (response[0] is the high byte).
    angle_travelled = get_angle();
    printf("Distance travelled = %d, angle travelled = %d\n", distance_travelled, angle_travelled);
    
    // turn
    
    // set both wheels to drive
    command[0] = 145u;
    command[1] =   0u;
    command[2] = 100u;
    command[3] = 0xFF;
    command[4] = 0x9C;
    retVal = write(mSerialPort, command, 5);
    CHECK_RETURN(retVal, "ERROR: write2");
    printf("Turning\n");
    
    sleep(1);
    
    // stop both wheels
    command[0] = 145u;
    command[1] =   0u;
    command[2] =   0u;
    command[3] =   0u;
    command[4] =   0u;
    retVal = write(mSerialPort, command, 5);
    CHECK_RETURN(retVal, "ERROR: write3");
    printf("Stopping\n");
    
//    usleep(10000); // 10 ms
    // read off the distance travelled (response[0] is the high byte).
    distance_travelled = get_distance();
    // read off the angle travelled (response[0] is the high byte).
    angle_travelled = get_angle();
    printf("Distance travelled = %d, angle travelled = %d\n", distance_travelled, angle_travelled);
    
    // set both wheels to drive
    command[0] = 145u;
    command[1] =   0u;
    command[2] = 100u;
    command[3] =   0u;
    command[4] = 100u;
    retVal = write(mSerialPort, command, 5);
    CHECK_RETURN(retVal, "ERROR: write2");
    printf("Driving\n");
    
    // wait
    //sleep(3);
    
    for (int i = 0; i < 30; i++)
    {
        usleep(30000); // 30 ms
        // read off the distance travelled (response[0] is the high byte).
        distance_travelled = get_distance();
        // read off the angle travelled (response[0] is the high byte).
        angle_travelled = get_angle();
        printf("Distance travelled = %d, angle travelled = %d\n", distance_travelled, angle_travelled);
    }
    
    // stop both wheels
    command[0] = 145u;
    command[1] =   0u;
    command[2] =   0u;
    command[3] =   0u;
    command[4] =   0u;
    retVal = write(mSerialPort, command, 5);
    CHECK_RETURN(retVal, "ERROR: write3");
    printf("Stopping\n");
    
//    usleep(10000); // 10 ms
    // read off the distance travelled (response[0] is the high byte).
    distance_travelled = get_distance();
    // read off the angle travelled (response[0] is the high byte).
    angle_travelled = get_angle();
    printf("Distance travelled = %d, angle travelled = %d\n", distance_travelled, angle_travelled);
    
    
    /*// read off the angle travelled (response[0] is the high byte).
    command[0] = 142u;
    command[1] =  20u;
    retVal = write(mSerialPort, command, 2);
    CHECK_RETURN(retVal, "ERROR: write5");
    usleep(500);
    retVal = read(mSerialPort, response, 2);
    CHECK_RETURN(retVal, "ERROR: read2");
    angle_travelled = response[1] + ((response[0] & 0x7f) << 8) - ((response[0] & 0x80) << 8);
    
    // print the results (result[0] is the high byte)
    printf("Distance = %d, angle = %d\n", distance_travelled, angle_travelled);*/
#endif
#ifdef TEST_2
    // ---------- TEST 2 ----------
    // tell roomba to drive forward 100 mm
    command[0]  = 152u; // script of length 13
    command[1]  =  13u;
    command[2]  = 145u; // drive forward @ 100 mm/s straight
    command[3]  =   0u;
    command[4]  = 100u;
    command[5]  =   0u;
    command[6]  = 100u;
    command[7]  = 156u; // wait until distance travelled is 100 mm
    command[8]  =   20u;
    command[9]  =  00u;
    command[10] = 145u; // stop driving
    command[11] =   0u;
    command[12] =   0u;
    command[13] =   0u;
    command[14] =   0u;
    retVal = write(mSerialPort, command, 15);
    CHECK_RETURN(retVal, "ERROR: write5");
   
   
    // read off the distance travelled.
    command[0] = 142u;
    command[1] =  19u;
    retVal = write(mSerialPort, command, 2);
    CHECK_RETURN(retVal, "ERROR: write6");
    retVal = read(mSerialPort, response, 2);
    CHECK_RETURN(retVal, "ERROR: read1");
    
    // print the results (result[0] is the high byte)
    distance_travelled = response[1] + ((response[0] & 0x7f) << 8) - ((response[0] & 0x80) << 8);
    printf("Distance (bytes read = %x %x) = %d\n", response[0], response[1], distance_travelled);
#endif
    
   return 0;
}
