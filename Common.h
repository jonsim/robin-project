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
#include <math.h>
// opencv -- this shouldn't really be here, but because marker data is who cares.
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>




/*-------------------- DEFINES  --------------------*/
// Program optimisation definitions
//#define SAFE_MIN_MAX

// Camera calibration/feature definitions
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define PIXEL_SIZE   0.104200f
#define FOCAL_LENGTH 120

// Things that should already be defined but for some reason aren't on the pandaboard's implementation of c :(
#ifndef UINT32_MAX
    #define UINT32_MAX 0xFFFFFFFF   // this should definitely come from stdint.h, but it doesn't seem to want to so lets make sure we have it.
#endif
#ifndef INT32_MAX
    #define INT32_MAX  0x7FFFFFFF   // See above.
#endif
#ifndef PI
    #define PI 3.141592654
#endif
#define RADTODEG(x) (x * (180.0/PI))
#define DEGTORAD(x) (x * (PI/180.0))
#define VECTOR_UNNORMALISED   0
#define VECTOR_AUTONORMALISED 1

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
    #if !(defined(MIN) && defined(MAX))
        // more efficient min/max but which suffer from double evaluation.
        #define MIN(a,b) (((a) < (b)) ? (a) : (b))
        #define MAX(a,b) (((a) > (b)) ? (a) : (b))
    #endif
#endif



/*-------------- CLASS DEFINITIONS --------------*/
struct Point2
{
    Point2 (const uint16_t x_init, const uint16_t y_init) : x(x_init), y(y_init) {}
    ~Point2 (void) {}
    uint16_t x, y;
};

struct Point2i
{
    Point2i  (const int x_init, const int y_init) : x(x_init), y(y_init) {}
    Point2i  (const int x_old,  const int y_old, const int displacement, const int angle)
    {
        x = x_old + ((int) (displacement * sin(DEGTORAD(angle))));
        y = y_old + ((int) (displacement * cos(DEGTORAD(angle))));
    }
    Point2i  (const Point2i& p_old, const int displacement, const int angle)
    {
        x = p_old.x + ((int) (((float) displacement) * sin(DEGTORAD((float) angle))));
        y = p_old.y + ((int) (((float) displacement) * cos(DEGTORAD((float) angle))));
    }
    ~Point2i (void) {}
    Point2i operator+ (const Point2i& other)
    {
        return Point2i(x + other.x, y + other.y);
    }
    Point2i operator- (const Point2i& other)
    {
        return Point2i(x - other.x, y - other.y);
    }
    bool operator== (const Point2i& other)
    {
        return ((x == other.x) && (y == other.y));
    }
    bool operator!= (const Point2i& other)
    {
        return ((x != other.x) || (y != other.y));
    }
    int x, y;
};

struct Point3
{
    Point3 (const float x_init, const float y_init, const float z_init) : x(x_init), y(y_init), z(z_init) {}
    ~Point3 (void) {}
    float x, y, z;
};

struct Vector2
{
    Vector2 (const float x_init, const float y_init, const int type_init=VECTOR_UNNORMALISED) : x(x_init), y(y_init), type(type_init)
    {
        if (type == VECTOR_AUTONORMALISED)
            normalise();
    }
    void normalise (void)
    {
        float m = sqrt(x*x + y*y);
        x = x / m;
        y = y / m;
    }
    float operator* (const Vector2& other)
    {
        return (x * other.x) + (y * other.y);
    }
    float angleTo (const Vector2& other)
    {
        float dot_product = (*this) * other;
        return RADTODEG(acos(dot_product));
    }
    Vector2 operator+ (const Vector2& other)
    {
        return Vector2(x + other.x, y + other.y, type & other.type);
    }
    float x, y;
    int type;
};

struct Vector3
{
    Vector3 (const float x_init, const float y_init, const float z_init) : x(x_init), y(y_init), z(z_init) {}
    ~Vector3 (void) {}
    float x, y, z;
};

struct MarkerData
{
    cv::Point3_<float> position;
    float              orientation;
};




/*-------------- FUNCTION DEFINITIONS --------------*/
sint8_t  make_sint8_t  (const uint8_t b);
uint16_t make_uint16_t (const uint8_t bh, const uint8_t bl);
sint16_t make_sint16_t (const uint8_t bh, const uint8_t bl);
float    euclidean_distance  (const Point2i& p1, const Point2i& p2);
float    euclidean_distance2 (const Point2i& p1, const Point2i& p2);
float    randNormallyDistributed (float mu, float sigma);
void     msleep        (const uint32_t msec);
int      _kbhit        (void);
std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems);
std::vector<std::string> split(const std::string& s, char delim);

#endif
