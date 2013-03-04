/**
 * @file    Common.h
 * @author  Jonathan Simmonds
 * @brief     Set of common functions, definitions and includes.
 */
#ifndef COMMON_H
#define COMMON_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h> // _kbhit
#include <sys/ioctl.h>  // _kbhit
#include <termios.h>    // _kbhit
#include <stropts.h>    // _kbhit




/*-------------------- DEFINES  --------------------*/
// Program optimisation definitions
//#define SAFE_MIN_MAX

// Camera calibration/feature definitions
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define PIXEL_SIZE   0.104200f
#define FOCAL_LENGTH 120

typedef unsigned char          bool_t;
typedef unsigned char          uint8_t;
typedef unsigned short int     uint16_t;
//typedef unsigned long  int     uint32_t;
typedef unsigned int           uint32_t;
//typedef unsigned long long int uint64_t;
typedef signed char            sint8_t;
typedef signed short int       sint16_t;
typedef signed long  int       sint32_t;
//typedef signed long long int   sint64_t;




/*--------------------  MACROS  --------------------*/
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

// Check annoying non-standard return values from Xn and report the error if necessary.
#define CHECK_RETURN_XN(rc, what)                                \
    if (rc != XN_STATUS_OK)                                      \
    {                                                            \
        printf("%s failed: %s\n", what, xnGetStatusString(rc));  \
        fflush(stdout);                                          \
        exit(EXIT_FAILURE);                                      \
    }

// Min/Max functions.
#ifdef SAFE_MIN_MAX
    // less efficient min/max which are safe.
    #define MIN(a,b)             \
       ({ __typeof__ (a) _a = (a); \
          __typeof__ (b) _b = (b); \
          _a < _b ? _a : _b; })
    #define MAX(a,b)               \
       ({ __typeof__ (a) _a = (a); \
          __typeof__ (b) _b = (b); \
          _a > _b ? _a : _b; })
#else
    // more efficient min/max but which suffer from double evaluation.
    #define MIN(a,b) (((a) < (b)) ? (a) : (b))
    #define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif




/*-------------- CLASS DEFINITIONS --------------*/
class Point2
{
public:
    Point2 (const uint16_t x_init, const uint16_t y_init) : x(x_init), y(y_init) {}
    ~Point2 (void) {}
    uint16_t x, y;
};

class Vector3
{
public:
    Vector3 (const float x_init, const float y_init, const float z_init) : x(x_init), y(y_init), z(z_init) {}
    ~Vector3 (void) {}
    float x, y, z;
};




/*-------------- FUNCTION DEFINITIONS --------------*/
sint8_t  make_sint8_t  (const uint8_t b);
uint16_t make_uint16_t (const uint8_t bh, const uint8_t bl);
sint16_t make_sint16_t (const uint8_t bh, const uint8_t bl);
void     msleep        (const uint32_t msec);
int      _kbhit        (void);

#endif
