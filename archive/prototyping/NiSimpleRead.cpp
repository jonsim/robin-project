/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>
#include <XnFPSCalculator.h>

#include <stdio.h>
#include <queue>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#ifndef NAN
    // define nan?
#endif
#ifndef INFINITY
    #define INFINITY (float) 1000000
#endif

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "CircularBuffer.h"

// prepare to include terribly written matplotpp library. making the extremely
// dangerous assumption that these warnings are not a problem (mostly
// uninitialized variable use, but some other nasty ones too).
#pragma warning(push)
#pragma warning( disable : 4018 )   // signed/unsigned mismatch
#pragma warning( disable : 4100 )   // unreferenced formal parameter
#pragma warning( disable : 4101 )   // unreferenced local variable
#pragma warning( disable : 4189 )   // local variable is initialized but not referenced
#pragma warning( disable : 4389 )   // signed/unsigned mismatch
#pragma warning( disable : 4244 )   // conversion from 'int' to 'float', possible loss of data
#pragma warning( disable : 4701 )   // potentially uninitialized local variable used
#pragma warning( disable : 4706 )   // assignment within conditional expression
#pragma warning( disable : 4715 )   // not all control paths return a value
#include "D:\Program Files (x86)\matplotpp\matplotpp.h"
#pragma warning(pop)


//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../../Data/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"
//#define SAFE_MIN_MAX
#define SAFE_PRINTF

#define INIT_FRAMES 50
#define CAPTURE_FRAMES 51
#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480
#define PIXEL_SIZE   0.104200f
#define FOCAL_LENGTH 120

#define DBSCAN_EPSILON       500 // mm
#define DBSCAN_MIN_POINTS      5

#define CLUSTERING_ENABLED
#define CLUSTER_SUBTRACTION_BUFFER
#define FULL_3D_PROJECTION

#ifdef CLUSTERING_ENABLED
    #define FRAME_RETENTION         10 // 1 < FRAME_RETENTION <= 255
    #define SUBSAMPLING_FACTOR       8
    #define DIFFERENCE_THRESHOLD    30 // mm
    #define MATPLOT_WINDOW_WIDTH   720
    #define MATPLOT_WINDOW_HEIGHT  600
#else
    #define FRAME_RETENTION          2 // 1 < FRAME_RETENTION <= 255
    #define SUBSAMPLING_FACTOR       1
    #define DIFFERENCE_THRESHOLD    30 // mm
    #define MATPLOT_WINDOW_WIDTH  1200
    #define MATPLOT_WINDOW_HEIGHT 1000
#endif

#define HIST_MAX_VALUE              5000 // mm
#define HIST_NEAR_RANGE_START        400 // mm
#define HIST_NEAR_RANGE_END          600 // mm
#define HIST_STATIC_PANIC_THRESHOLD  500 // this many readings in the near range will cause instant panic
#define HIST_DYNAMIC_PANIC_THRESHOLD 100 // this many readings suddenly transitioning out of the near range will cause panic

#define STREAM_WINDOW_NAME "DepthImage"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)                                            \
    if (rc != XN_STATUS_OK)                                            \
    {                                                                \
        printf("%s failed: %s\n", what, xnGetStatusString(rc));        \
        return rc;                                                    \
    }
#ifdef SAFE_MIN_MAX
    // less efficient min/max which are safe
    #define MIN(a,b)             \
       ({ __typeof__ (a) _a = (a); \
          __typeof__ (b) _b = (b); \
          _a < _b ? _a : _b; })
    #define MAX(a,b)               \
       ({ __typeof__ (a) _a = (a); \
          __typeof__ (b) _b = (b); \
          _a > _b ? _a : _b; })
#else
    // efficient min/max which suffer from double evaluation
    #define MIN(a,b) (((a) < (b)) ? (a) : (b))
    #define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifdef SAFE_PRINTF
    #define printf(fmt, ...)                            \
        {                                               \
            char printf_buffer[1024];                   \
            sprintf(printf_buffer, fmt, __VA_ARGS__);   \
            OutputDebugStringA(printf_buffer);          \
        }
#endif




//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;
typedef unsigned short uint16_t;
//typedef unsigned long  uint32_t;
typedef signed long sint32_t;
typedef unsigned char  uint8_t;




void createGrayscaleDepthImage (cv::Mat* dst, const XnDepthPixel* src)
{
    static const uint16_t max_depth      = 5000u;
    static const float    scaling_factor = 255.0f / max_depth;
    uint32_t one_step_offset;
    uint16_t v, x, y;
    uint8_t  gs;
    
    for (y=0, one_step_offset=0; y < IMAGE_HEIGHT; y++)
    {
        for (x=0; x < IMAGE_WIDTH; x++, one_step_offset++)
        {
            v  = (uint16_t) src[one_step_offset];
            gs = (uint8_t)  (v * scaling_factor);
            dst->data[one_step_offset] = gs;
        }
    }
}



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

class BinaryMarker
{
public:
    BinaryMarker (uint32_t size)
    {
        mBitSize  = size;
        mByteSize = (uint32_t) ceil(size / 8.0f);
        A = (uint8_t*) calloc(mByteSize, sizeof(uint8_t));
    }

    ~BinaryMarker (void)
    {
        free(A);
    }

    void set (const uint32_t i, const uint8_t state)
    {
        if (i > mBitSize)
            return;
        uint32_t index = i >> 3;                    // i / 8
        uint8_t  pos   = (uint8_t) (i - (index*8)); // i % 8
        uint8_t  mask  = 1 << pos;
        if (state)
            A[index] |= mask;   // set the bit to 1
        else
            A[index] &= !mask;  // set the bit to 0
    }

    uint8_t get (const uint32_t i) const
    {
        if (i > mBitSize)
            return 0;
        uint32_t index = i >> 3;                    // i / 8
        uint8_t  pos   = (uint8_t) (i - (index*8)); // i % 8
        uint8_t  mask  = 1 << pos;
        return (A[index] & mask) >> pos;
    }

    uint32_t hammingWeight (void) const
    {
        const uint8_t mask_1  = 0x55; // 01010101
        const uint8_t mask_2  = 0x33; // 00110011
        const uint8_t mask_4  = 0x0f; // 00001111
        uint8_t  byteAccumulator = 0;
        uint32_t fullAccumulator = 0;
        uint32_t i;

        for (i = 0; i < mByteSize; i++)
        {
            //This is a naive implementation, shown for comparison,
            //and to help in understanding the better functions.
            //It uses 24 arithmetic operations (shift, add, and).
            //Refer to http://en.wikipedia.org/wiki/Hamming_weight for better implementations
            byteAccumulator = A[i];
            byteAccumulator = (byteAccumulator & mask_1) + ((byteAccumulator >> 1) & mask_1); // put count of each 2 bits into those 2 bits 
            byteAccumulator = (byteAccumulator & mask_2) + ((byteAccumulator >> 2) & mask_2); // put count of each 4 bits into those 4 bits 
            byteAccumulator = (byteAccumulator & mask_4) + ((byteAccumulator >> 4) & mask_4); // put count of each 8 bits into those 8 bits
            fullAccumulator += byteAccumulator;
        }

        return fullAccumulator;
    }

private:
    uint8_t* A;
    uint32_t mBitSize;
    uint32_t mByteSize;
};

class Histogram
{
public:
    Histogram (void) : max_value(HIST_MAX_VALUE)
    {
        hist = (uint16_t*) calloc(max_value+1, sizeof(uint16_t));
    }

    Histogram (const uint16_t maximum_value) : max_value(maximum_value)
    {
        hist = (uint16_t*) calloc(max_value+1, sizeof(uint16_t));
    }

    Histogram (const uint16_t* data, const uint32_t data_count, const uint16_t maximum_value) : max_value(maximum_value)
    {
        uint32_t one_step_index;

        hist = (uint16_t*) calloc(max_value+1, sizeof(uint16_t));
        for (one_step_index = 0; one_step_index < data_count; one_step_index++)
            hist[MIN(data[one_step_index], max_value)]++;
    }

    ~Histogram (void)
    {
        free(hist);
    }

    // note this uses the same maximum_value. if you need a different maximum value, make a new histogram.
    void rebuild (const uint16_t* data, const uint32_t data_count)
    {
        uint32_t one_step_index;

        memset(hist, 0, (max_value+1) * sizeof(uint16_t));
        for (one_step_index = 0; one_step_index < data_count; one_step_index++)
            hist[MIN(data[one_step_index], max_value)]++;
    }

    inline uint16_t get (const uint16_t value) const
    {
        return hist[MIN(value, max_value)];
    }

    // inclusive
    inline uint32_t getRange (const uint16_t starting_value, uint16_t ending_value) const
    {
        uint16_t i;
        uint32_t accumulator = 0;

        ending_value = MIN(ending_value, max_value);
        for (i = starting_value; i <= ending_value; i++)
            accumulator += hist[i];
        return accumulator;
    }

private:
    uint16_t max_value;
    uint16_t* hist;
};

class FrameBuffer
{
public:
    FrameBuffer  (const uint16_t xres, const uint16_t yres, const uint16_t sampling_factor, const uint8_t frame_retention) :
      input_xres(xres),
      input_yres(yres),
      input_frame_res(input_xres * input_yres),
      resampling_factor(sampling_factor),
      output_xres(xres / sampling_factor),
      output_yres(yres / sampling_factor),
      output_frame_res(output_xres * output_yres),
      pixel_size((PIXEL_SIZE * IMAGE_WIDTH * 2) / output_xres),
      projection_constant(pixel_size / FOCAL_LENGTH),
      retention(frame_retention),
      histogramNearRangeCount(0),
      histogramErrorRangeCount(0),
      lead_in_count(frame_retention),
      dataBuffer(frame_retention),   // TODO not because of the way I use the buffer the extra space which is supposed to be free isn't.
                                     // this is not a problem but means a frame_retention of < 2 is not possible. Also the pre-load would break.
      histogramBuffer(frame_retention), // same problem as above
      subtractionBuffer(output_frame_res)
    {
        uint8_t i;

        // Create the raw data buffer (which will hold the raw frame data, managed by a CircularBuffer of pointers).
        rawDataBuffer = (uint16_t*) malloc(sizeof(uint16_t) * output_frame_res * frame_retention);
        if (rawDataBuffer == NULL)
            printf("Couldn't allocate FrameBuffer rawDataBuffer memory\n");
        
        // pre-load buffer. this will just have gibberish in for the first frame_retention frames, hence the
        // use of a lead_in_count variable to refuse all access requests before then.
        for (i = 0; i < frame_retention; i++)
            dataBuffer.add(&(rawDataBuffer[output_frame_res * i]));

        // Create the raw histogram buffer (which will hold histogram representations of every frame, again managed by a CircularBuffer).
        rawHistogramBuffer = new Histogram[frame_retention];

        // pre-load the CircularBuffer.
        for (i = 0; i < frame_retention; i++)
            histogramBuffer.add(&(rawHistogramBuffer[i]));
    }

    ~FrameBuffer (void)
    {
        free(rawDataBuffer);
    }

    FrameBuffer& operator= (const FrameBuffer& other)
    {
        if (this != &other)
        {
            printf("FrameBuffer Assignment Operator is disallowed.\n");
            exit(EXIT_FAILURE);
        }
        return *this;
    }

    void insert (const uint16_t* data)
    {
        uint16_t data_x, data_y;
        uint32_t one_step_index, sample_step_index;

        // rotate the buffer round one step
        uint16_t* frame = *(dataBuffer.add());

        // if we're still warming up, decrement the counter
        if (lead_in_count > 0)
            lead_in_count--;

        // copy the data into the frame buffer, sampling every sampling_factor pixels. this will overwrite whatever's in the buffer already.
        for (one_step_index=0, sample_step_index=0, data_y=0; data_y < input_yres; data_y+=resampling_factor, sample_step_index=(data_y*input_xres))
            for (                                   data_x=0; data_x < input_xres; data_x+=resampling_factor, sample_step_index+=resampling_factor, one_step_index++)
                frame[one_step_index] = data[sample_step_index];

        // build a histogram from the newly sampled data.
        Histogram* hist = *(histogramBuffer.add());
        hist->rebuild(frame, output_frame_res);

        // PANIC?!?!????!?!???!??!?!!!
        if (lead_in_count == 0)
        {
            uint32_t oldHistogramErrorRangeCount = histogramErrorRangeCount;
            uint32_t oldHistogramNearRangeCount  = histogramNearRangeCount;
            histogramErrorRangeCount = hist->get(0);
            histogramNearRangeCount  = hist->getRange(HIST_NEAR_RANGE_START, HIST_NEAR_RANGE_END);

            // check for static panic threshold
            if (histogramNearRangeCount > HIST_STATIC_PANIC_THRESHOLD)
                printf("STATIC PANIC! :O\n");

            // check for dynamic panic threshold
            /*if (histogramNearRangeCount > (oldHistogramNearRangeCount + HIST_DYNAMIC_PANIC_THRESHOLD))
                printf("DYNAMIC PANIC! :O\n");*/
            if (histogramNearRangeCount  < (oldHistogramNearRangeCount  - HIST_DYNAMIC_PANIC_THRESHOLD) &&
                histogramErrorRangeCount > (oldHistogramErrorRangeCount + HIST_DYNAMIC_PANIC_THRESHOLD)   )
            {
                printf("DYNAMIC PANIC! :O\n");
            }
        }
    }

    // where frame_number is 0...retention-1 and 0 is the most recent frame.
    // this can return null if the index is invalid or we are not yet warmed up.
    uint16_t* retrieve (const uint8_t frame_number) const
    {
        if ((frame_number < retention) && (lead_in_count == 0))
            return *(dataBuffer.get(frame_number));
        else
            return NULL;
    }

    void fillSubtractionBuffer (const uint8_t frame_number1, const uint8_t frame_number2)
    {
        sint32_t  difference;
        uint32_t  one_step_index;
        uint16_t* frame1 = retrieve(frame_number1);
        uint16_t* frame2 = retrieve(frame_number2);

        if (frame1 == NULL || frame2 == NULL)
            return;

        for (one_step_index = 0; one_step_index < output_frame_res; one_step_index++)
        {
            difference = ((sint32_t) frame1[one_step_index]) - ((sint32_t) frame2[one_step_index]);
            if (abs(difference) > DIFFERENCE_THRESHOLD)
                subtractionBuffer.set(one_step_index, 1);
            else
                subtractionBuffer.set(one_step_index, 0);
        }
    }

    // transformation is applied to frame1 before subtraction with frame2
    void fillSubtractionBufferWithTransformation (const uint8_t frame_number1, const uint8_t frame_number2, const Vector3 translation, const float y_rotation)
    {
        sint32_t  difference;
        uint32_t  one_step_index;
        uint16_t* frame1 = retrieve(frame_number1);
        uint16_t* frame2 = retrieve(frame_number2);

        if (frame1 == NULL || frame2 == NULL)
            return;

        for (one_step_index = 0; one_step_index < output_frame_res; one_step_index++)
        {
            difference = ((sint32_t) frame1[one_step_index]) - ((sint32_t) frame2[one_step_index]);
            if (abs(difference) > DIFFERENCE_THRESHOLD)
                subtractionBuffer.set(one_step_index, 1);
            else
                subtractionBuffer.set(one_step_index, 0);
        }
    }

    void projectAllPoints (const uint8_t frame_number, dvec* x, dvec* y, dvec* z) const
    {
        const static uint16_t half_xres = output_xres/2;
        const static uint16_t half_yres = output_yres/2;
        uint16_t  xi, yi;
        uint32_t  one_step_index;
        double    X, Y, Z, Z_projection_constant;

        uint16_t* frame = retrieve(frame_number);

        if (frame == NULL)
            return;
        for (yi = 0, one_step_index = 0; yi < output_yres; yi++)
        {
            for (xi = 0; xi < output_xres; xi++, one_step_index++)
            {
                Z = frame[one_step_index];
#ifdef FULL_3D_PROJECTION
                Z_projection_constant = Z * projection_constant;
#else
                Z_projection_constant = 20;
#endif
                X = (xi - half_xres) * Z_projection_constant;
                Y = (half_yres - yi) * Z_projection_constant;
                x->push_back(X);
                y->push_back(Y);
                z->push_back(Z);
            }
        }
    }

    void projectSubtractionBuffer (const uint8_t frame_number, dvec* x, dvec* y, dvec* z) const
    {
        const static uint16_t half_xres = output_xres/2;
        const static uint16_t half_yres = output_yres/2;
        uint16_t  xi, yi;
        uint32_t  one_step_index;
        double    X, Y, Z, Z_projection_constant;

        uint16_t* frame = retrieve(frame_number);

        if (frame == NULL)
            return;
        for (yi = 0, one_step_index = 0; yi < output_yres; yi++)
        {
            for (xi = 0; xi < output_xres; xi++, one_step_index++)
            {
                if (subtractionBuffer.get(one_step_index))
                {
                    Z = frame[one_step_index];
#ifdef FULL_3D_PROJECTION
                    Z_projection_constant = Z * projection_constant;
#else
                    Z_projection_constant = 20;
#endif
                    X = (xi - half_xres) * Z_projection_constant;
                    Y = (half_yres - yi) * Z_projection_constant;
                    x->push_back(X);
                    y->push_back(Y);
                    z->push_back(Z);
                }
            }
        }
    }

    float euclidian_distance2 (const uint8_t frame_number, const Point2 p1, const Point2 p2) const
    {
        const static uint16_t half_xres = output_xres/2;
        const static uint16_t half_yres = output_yres/2;
        uint16_t* frame = retrieve(frame_number);

        // TODO this entire function can be heavily optimised
        uint16_t p1_Z = frame[p1.x + (p1.y * output_xres)];
        uint16_t p2_Z = frame[p2.x + (p2.y * output_xres)];
        float dZ = (float) (p1_Z - p2_Z);

#ifdef FULL_3D_PROJECTION
        uint32_t p1_X = (p1.x - half_xres) * p1_Z;     // * projection_constant
        uint32_t p2_X = (p2.x - half_xres) * p2_Z;     // * projection_constant
        float dX = (float) (p1_X - p2_X) * projection_constant;

        uint32_t p1_Y = (half_yres - p1.y) * p1_Z;     // * projection_constant
        uint32_t p2_Y = (half_yres - p2.y) * p2_Z;     // * projection_constant
        float dY = (float) (p1_Y - p2_Y) * projection_constant;
#else
        uint32_t p1_X = (p1.x - half_xres) * 20;     // * projection_constant
        uint32_t p2_X = (p2.x - half_xres) * 20;     // * projection_constant
        float dX = (float) (p1_X - p2_X);

        uint32_t p1_Y = (half_yres - p1.y) * 20;     // * projection_constant
        uint32_t p2_Y = (half_yres - p2.y) * 20;     // * projection_constant
        float dY = (float) (p1_Y - p2_Y);
#endif

        return (dX * dX) + (dY * dY) + (dZ * dZ);
    }

    // returns true if the distance between two given points is less than the threshold.
    // crucially this is an easier question to answer than flat out asking the distance between two points.
    bool euclidian_distance2_thresholded (const uint8_t frame_number, const Point2 p1, const Point2 p2, const uint32_t threshold2) const
    {
        const static uint16_t half_xres = output_xres/2;
        const static uint16_t half_yres = output_yres/2;
        uint16_t* frame = retrieve(frame_number);

        // TODO this entire function can be heavily optimised
        uint16_t p1_Z = frame[p1.x + (p1.y * output_xres)];
        uint16_t p2_Z = frame[p2.x + (p2.y * output_xres)];
        float dZ = (float) (p1_Z - p2_Z);
        
#ifdef FULL_3D_PROJECTION
        uint32_t p1_X = (p1.x - half_xres) * p1_Z;     // * projection_constant
        uint32_t p2_X = (p2.x - half_xres) * p2_Z;     // * projection_constant
        float dX = (float) (p1_X - p2_X) * projection_constant;

        uint32_t p1_Y = (half_yres - p1.y) * p1_Z;     // * projection_constant
        uint32_t p2_Y = (half_yres - p2.y) * p2_Z;     // * projection_constant
        float dY = (float) (p1_Y - p2_Y) * projection_constant;
#else
        uint32_t p1_X = (p1.x - half_xres) * 20;     // * projection_constant
        uint32_t p2_X = (p2.x - half_xres) * 20;     // * projection_constant
        float dX = (float) (p1_X - p2_X);

        uint32_t p1_Y = (half_yres - p1.y) * 20;     // * projection_constant
        uint32_t p2_Y = (half_yres - p2.y) * 20;     // * projection_constant
        float dY = (float) (p1_Y - p2_Y);
#endif

        return (((dX * dX) + (dY * dY) + (dZ * dZ)) < threshold2);
    }

    
    const uint16_t output_xres;
    const uint16_t output_yres;
    const uint32_t output_frame_res;
    BinaryMarker subtractionBuffer;

private:
    const uint16_t input_xres;
    const uint16_t input_yres;
    const uint32_t input_frame_res;
    const uint16_t resampling_factor;
    const float    pixel_size;
    const float    projection_constant;

    uint32_t histogramNearRangeCount;
    uint32_t histogramErrorRangeCount;

    const uint8_t  retention;
    uint8_t        lead_in_count;
    CircularBuffer<uint16_t*> dataBuffer;
    uint16_t*      rawDataBuffer;
    CircularBuffer<Histogram*> histogramBuffer;
    Histogram*     rawHistogramBuffer;

};

void DBSCAN_regionQuery (const FrameBuffer* fb, const Point2 p, std::vector<Point2>* region, const uint32_t epsilon, const uint32_t epsilon2)
{
    uint16_t x, y;
    uint32_t one_step_index;

    // This probably shouldn't be done here, but it is far easier/safer than remembering to do it before calling.
    region->clear();

    // TODO this can be heavily optimised, we definitely don't need to search all the points, we know so much
    //      about the structure of the data we're searching.
    // for each point, calculate the distance from the query point and return those that fall within a certain distance.
    for (y = 0, one_step_index = 0; y < fb->output_yres; y++)
    {
        for (x = 0; x < fb->output_xres; x++, one_step_index++)
        {
#ifdef CLUSTER_SUBTRACTION_BUFFER
            if (fb->subtractionBuffer.get(one_step_index) && (x != p.x || y != p.y))
#else
            if (x != p.x || y != p.y)
#endif
            {
                const Point2 cp(x, y);
                // TODO THIS IS A MASSIVE HACK FOR NOW, WE DONT WANT TO FORCE FRAME 0 REALLY :(
                //if (fb->euclidian_distance2(0, p, cp) < epsilon2)
                if (fb->euclidian_distance2_thresholded(0, p, cp, epsilon2))
                    region->push_back(cp);
            }
        }
    }

    //printf("found %d elements in (%d, %d)'s epsilon-neighbourhood\n", region->size(), p.x, p.y);
}

void DBSCAN_expandCluster (const FrameBuffer* fb, const Point2 p, BinaryMarker* visited, uint8_t* clusters, const uint8_t cluster_index, std::vector<Point2>* currentNeighbourhood, const uint32_t epsilon, const uint32_t epsilon2, const uint32_t min_points)
{
    uint32_t i, j;
    uint32_t p_index, cp_index;
    std::vector<Point2> currentSubNeighbourhood;

    // Add p to the current cluster.
    p_index = p.x + (p.y * fb->output_xres);
    clusters[p_index] = cluster_index;

    // For each point cp in currentNeighbourhood
    for (i = 0; i < currentNeighbourhood->size(); i++)
    {
        Point2 cp = (*currentNeighbourhood)[i];
        cp_index = (cp.y * fb->output_xres) + cp.x;
        
        // if cp is not yet a member of any cluster
        if (clusters[cp_index] == 0 || clusters[cp_index] == 0xFF)
            // add cp to the current cluster.
            clusters[cp_index] = cluster_index;

        // if cp is not visited
        if (!(visited->get(cp_index)))
        {
            // mark cp as visited
            visited->set(cp_index, 1u);

            // currentSubNeighbourhood = regionQuery(cp)
            DBSCAN_regionQuery(fb, cp, &currentSubNeighbourhood, epsilon, epsilon2);

            // if the current subneighbourhood is sufficiently dense.
            if (currentSubNeighbourhood.size() >= min_points)
                // currentNeighbourhood += currentSubNeighbourhood.
                for (j = 0; j < currentSubNeighbourhood.size(); j++)
                    currentNeighbourhood->push_back(currentSubNeighbourhood[j]);
        }
    }
}

uint8_t DBSCAN (const FrameBuffer* fb, uint8_t* clusters, const uint32_t epsilon, const uint32_t min_points)
{
    const uint32_t epsilon2 = epsilon * epsilon;
    uint8_t  cluster_counter = 0;
    uint16_t x, y;
    uint32_t one_step_index;
    uint32_t i;
    std::vector<Point2> currentNeighbourhood;
    BinaryMarker        visited(fb->output_frame_res);  // inited to 0. this only needs to be clusters_size, but we would have no way of efficiently indexing them.
    
    // zero out the clusters array
    for (i = 0; i < fb->output_frame_res; i++)
        clusters[i] = 0;

    for (y = 0, one_step_index = 0; y < fb->output_yres; y++)
    {
        for (x = 0; x < fb->output_xres; x++, one_step_index++)
        {
            // first check if the pixel is set in the subtraction buffer (implying it has moved in the time set by the frameSubtract operation).
            // then check we have not already visited the point (which could have happened in a DBSCAN_expandCluster operation).
#ifdef CLUSTER_SUBTRACTION_BUFFER
            if (fb->subtractionBuffer.get(one_step_index) && !visited.get(one_step_index))
#else
            if (!visited.get(one_step_index))
#endif
            {
                // form the point.
                Point2 p(x, y);

                // mark the point as visited.
                visited.set(one_step_index, 1u);

                // run a region query on the point.
                DBSCAN_regionQuery(fb, p, &currentNeighbourhood, epsilon, epsilon2);

                // check if the point's surrounding area is sufficiently dense.
                if (currentNeighbourhood.size() >= min_points)
                {
                    // make a new cluster (if we have made too many, panic).
                    cluster_counter++;
                    if (cluster_counter > 100)
                        printf("ERROR: TOO MANY CLUSTERS, PREPARE FOR CRASHES\n");

                    // look to see if we can expand that cluster (by way of density-reachable points).
                    DBSCAN_expandCluster(fb, p, &visited, clusters, cluster_counter, &currentNeighbourhood, epsilon, epsilon2, min_points);
                }
                else
                {
                    // otherwise mark it down as noise.
                    clusters[one_step_index] = 0xFF;
                }
            }
        }
    }
    return cluster_counter;
}

int is_run=1;

XnStatus nRetVal = XN_STATUS_OK;
Context context;
ScriptNode scriptNode;
EnumerationErrors errors;
DepthGenerator depth;
DepthMetaData depthMD;
cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
ImageGenerator color;
ImageMetaData  colorMD;
cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
XnFPSData xnFPS;
FrameBuffer fred(IMAGE_WIDTH, IMAGE_HEIGHT, SUBSAMPLING_FACTOR, FRAME_RETENTION);



class MP : public MatPlot
{ 
    void DISPLAY()
    {
        dvec x,y,z,x1,y1,z1;
        ivec c;

        if (is_run == 1)
            nRetVal = context.WaitOneUpdateAll(depth);
        if (nRetVal != XN_STATUS_OK)
        {
            printf("  UpdateData failed (but still continuing): %s\n", xnGetStatusString(nRetVal));
        }
        else
        {
            xnFPSMarkFrame(&xnFPS);

            depth.GetMetaData(depthMD);
            const XnDepthPixel* depthData = depthMD.Data();

            fred.insert(depthData);

#ifdef CLUSTERING_ENABLED
            fred.fillSubtractionBuffer(0, FRAME_RETENTION-2);
            fred.projectSubtractionBuffer(0, &x, &y, &z);

            static uint8_t* clusters = (uint8_t*) malloc(fred.output_frame_res * sizeof(uint8_t));
            uint8_t cluster_count = DBSCAN(&fred, clusters, DBSCAN_EPSILON, DBSCAN_MIN_POINTS);
            c.clear();
            for (uint32_t one_step_index = 0; one_step_index < fred.output_frame_res; one_step_index++)
                if (fred.subtractionBuffer.get(one_step_index))
                    c.push_back(clusters[one_step_index]);

            axis(1000,3000,-1000,1000,-500,1500);
            plot3(z,x,y,c);
#else
            fred.projectAllPoints(0, &x, &y, &z);
            axis(1000,3000,-1000,1000,-500,1500);
            plot3(z,x,y);
#endif

            /*createGrayscaleDepthImage(&depthImage, depthData);
        
            if (depthImage.empty())
                printf("ERROR: Image was empty, not displaying :(\n");
            else
                cv::imshow(STREAM_WINDOW_NAME, depthImage);
            cvWaitKey(5);*/
        
#ifdef CLUSTERING_ENABLED
            printf("  Frame %d, FPS: %.3f, SB-Sum: %d, Cluster Count: %d\n", depthMD.FrameID(), xnFPSCalc(&xnFPS), fred.subtractionBuffer.hammingWeight(), cluster_count);
#else
            printf("  Frame %d, FPS: %.3f\n", depthMD.FrameID(), xnFPSCalc(&xnFPS));
#endif
        }
    }
} mp;

void display(){ mp.display(); }
void reshape(int w,int h){ mp.reshape(w,h); }
void idle( void ){ 
    glutPostRedisplay(); 
    Sleep(10);
}
void mouse(int button, int state, int x, int y ){ mp.mouse(button,state,x,y); }
void motion(int x, int y ){mp.motion(x,y); }
void passive(int x, int y ){mp.passivemotion(x,y); }
void keyboard(unsigned char key, int x, int y){
    mp.keyboard(key, x, y); 
    if(key=='r'){ if(is_run==0){is_run=1;}else{is_run=0;}}
}


/// @brief  Creates a colour image (as a cv::Mat) representation of the depth data in the given array.
/// @param  dst   The cv::Mat into which the colour representation will be saved. The size of this
///               is defined by the IMAGE_HEIGHT and IMAGE_WIDTH values. The cv::Mat needs to be
///               in the CV_8UC3 mode (8 bit RGB).
/// @param  src   A pointer to the uint16_t array of depth data. The size of this is defined by the
///               IMAGE_HEIGHT and IMAGE_WIDTH values.
void createColourDepthImage (cv::Mat* dst, const uint16_t* src)
{
    static const float scaling_factor = 4800/1530;
    uint16_t x=0, y=0;
    uint32_t one_step_offset=0, three_step_offset=0;
    uint16_t v;
    uint8_t  r, g, b;
    uint8_t* loc;
    
    for (y=0, one_step_offset=0, three_step_offset=0; y < IMAGE_HEIGHT; y++)
    {
        for (x = 0; x < IMAGE_WIDTH; x++, one_step_offset++, three_step_offset+=3)
        {
            v = (uint16_t) src[one_step_offset];
            
            if (v == 0)
            {
                r = 0;
                g = 0;
                b = 0;
            }
            else
            {
                v -= 400;
                v = (v < 4800) ? (uint16_t) (v / scaling_factor) : 1530u;
                // H' takes values between 0-1530
                // H' =    0- 255  RGB=   255, 0-255, 0
                // H' =  255- 510  RGB= 255-0,   255, 0
                // H' =  510- 765  RGB=     0,   255, 0-255
                // H' =  765-1020  RGB=     0, 255-0, 255
                // H' = 1020-1275  RGB= 0-255,     0, 255
                // H' = 1275-1530  RGB=   255,     0, 255-0
                
                if (v < 255)
                {
                    r = 255u;
                    g = (uint8_t) v;  // g increases to 255
                    b =   0u;
                }
                else if (v < 510)
                {
                    r = (uint8_t) (510 - v);  // r falls to 0
                    g = 255u;
                    b =   0u;
                }
                else if (v < 765)
                {
                    r =   0u;
                    g = 255u;
                    b = (uint8_t) (v - 510);  // b increases to 255
                }
                else if (v < 1020)
                {
                    r =   0u;
                    g = (uint8_t) (1020 - v);  // g falls to 0
                    b = 255u;
                }
                else if (v < 1275)
                {
                    r = (uint8_t) (v - 1020);  // r increases to 255
                    g =   0u;
                    b = 255u;
                }
                else  // v <= 1530
                {
                    r = 255u;
                    g =   0u;
                    b = (uint8_t) (1530 - v);  // b falls to 0
                }
            }
            
            loc = dst->data + three_step_offset;
            loc[0] = b;
            loc[1] = g;
            loc[2] = r;
        }
    }
}





cv::Mat kind_of_fourier_it_up (cv::Mat* src)
{
    cv::Mat padded;                            //expand input image to optimal size
    int m = cv::getOptimalDFTSize( src->rows );
    int n = cv::getOptimalDFTSize( src->cols ); // on the border add zero values
    cv::copyMakeBorder(*src, padded, 0, m - src->rows, 0, n - src->cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    cv::dft(complexI, complexI);            // this way the result may fit in the source matrix

    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    cv::split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    cv::magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    cv::Mat magI = planes[0];

    magI += cv::Scalar::all(1);                    // switch to logarithmic scale
    cv::log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(cv::Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    cv::Mat q0(magI, cv::Rect(0,   0, cx, cy));  // Top-Left - Create a ROI per quadrant
    cv::Mat q1(magI, cv::Rect(cx,  0, cx, cy));  // Top-Right
    cv::Mat q2(magI, cv::Rect(0,  cy, cx, cy));  // Bottom-Left
    cv::Mat q3(magI, cv::Rect(cx, cy, cx, cy));  // Bottom-Right

    cv::Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    cv::normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                            // viewable image form (float between values 0 and 1).

    return magI;
    //cv::imshow("Input Image"       , *src   );    // Show the result
    //cv::imshow("spectrum magnitude", magI);
    //cv::waitKey();
}

void kind_of_reproject_strip (cv::Mat* m, int y_start, int y_end)
{
    int x, y, i;
    int xres = m->size().width;
//    int yres = m->size().height;

    printf("X_projected,Z_projected\n");
    for (y = y_start; y < y_end; y++)
    {
        for (x = 0; x < xres; x++, i++)
        {
            int X, Z;

            Z = m->data[(y * xres) + x] * (10000 / 255);
            X = (int) ((x - xres/2) * Z * (((PIXEL_SIZE * IMAGE_WIDTH * 2) / xres) / FOCAL_LENGTH));
            printf("%d,%d\n", X, Z);
        }
    }
}

void kind_of_normalise (cv::Mat* m)
{
    int i, v;
    int size = m->size().area();

    for (i = 0; i < size; i++)
    {
        v = m->data[i] * (10000 / 255);
        if (v > 2000)
            m->data[i] = 0u;
        else
            m->data[i] = (uint8_t) (v / (2000 / 255));
    }
}


void kind_of_letterbox (cv::Mat* m)
{
    int x, y, i;
    int xres = m->size().width;
    int yres = m->size().height;

    for (i = 0, y = 0; y < yres; y++)
    {
        for (   x = 0; x < xres; x++, i++)
        {
            if (y < 120 || y > 360)
                m->data[i] = 0;
        }
    }
}


void kind_of_subsample (cv::Mat* dst, cv::Mat* src, int x_step, int y_step, int y_start, int y_end)
{
    int x, y, dst_i;
    int xres = src->size().width;
//    int yres = src->size().height;

    for (dst_i=0, y=y_start; y < y_end; y += y_step)
    {
        for (x=0; x < xres; x += x_step, dst_i++)
        {
            // sum all substep pixels
            int substep_total = 0;
            int substep_nzero = 0;
            for (int y_prime = 0; y_prime < y_step; y_prime++)
            {
                for (int x_prime = 0; x_prime < x_step; x_prime++)
                {
                    int v = src->data[((y + y_prime) * xres) + (x + x_prime)];
                    if (v != 0)
                    {
                        substep_nzero++;
                        substep_total += v;
                    }
                }
            }

            dst->data[dst_i] = (substep_nzero == 0) ? 0u : (uint8_t) (substep_total / substep_nzero);
        }
    }
}


void opencv_depthy_tests (void)
{
    cv::Mat images[10];
    for (int i = 0; i < 10; i++)
    {
        // create the filename
        char infilename[256];
        sprintf(infilename, "D:\\Users\\Jon\\Dropbox\\thesis\\depth_images\\output_frame_%d_gs.png", i);
        char otfilename[256];
        sprintf(otfilename, "D:\\Users\\Jon\\Dropbox\\depth_image_haar\\negative_%d.png", i);
        char windowname[256];
        sprintf(windowname, "Image %d", i);

        // load the file and create appropriate storage containers
        images[i] = cv::imread(infilename, CV_LOAD_IMAGE_GRAYSCALE);
        if (!images[i].data)
        {
            printf("SHIT NULLS EVERYWHERE PANIC LIKES ITS ninteen sixty FUCK\n");
            return;
        }
        cv::Mat small(  48,  64, CV_8UC1);
        cv::Mat medium(240, 320, CV_8UC1);

        // do imagey things
        kind_of_normalise(&images[i]);
        kind_of_subsample(&small,  &images[i], 10, 5, 120, 360);
        kind_of_subsample(&medium, &images[i],  2, 1, 120, 360);

        // do naughty things
        //kind_of_reproject_strip(&small, 44, 52);
        //cv::Mat furry = kind_of_fourier_it_up(&medium);

        // save
        cv::imwrite(otfilename, small);

        // display
        cv::imshow(windowname, medium);
    }
    cv::waitKey();

    return;
}
















#define MARKER_WIDTH   50  // mm
#define MARKER_HEIGHT  50  // mm
#define MARKER_EPSILON 20  // mm
#define HAAR_SCALING   1.2  // default 1.1, larger numbers perform faster but may miss features.
#define HAAR_NEIGHBOURS 3
#define MARKER_EXTRACTION_GS_THRESHOLD   64     // the gs cutoff for binarisation.
#define MARKER_EXTRACTION_SIZE_THRESHOLD 80     // any pixel blobs less than this are removed.
cv::CascadeClassifier haar_cascade;
std::string cascade_name = "D:\\Users\\Jon\\Dropbox\\project_haar\\cascade2xml\\haarcascade_marker2_16.xml";
struct MarkerData
{
    cv::Point3_<float> position;
    float              orientation;
};


float cv_euclidean_distance2 (cv::Point_<int> p1, cv::Point_<int> p2)
{
    float dx = (float) p1.x - p2.x;
    float dy = (float) p1.y - p2.y;
    return dx*dx + dy*dy;
}


// TODO: we can output the region areas from this function but opencv wants to be a little diva and fuck up the heap when we try...
int connected_component_labelling (cv::Mat& src, cv::Mat& dst)
{
    int x, y, c_index, n_index, w_index, n_label, w_label;
    int xres = src.cols, yres = src.rows;
    int region_count = 0;
    int i;
    uint8_t region_label;
    std::vector<std::pair<int, int>> regions_equivalence;

    //printf("executing first pass\n");
    for (c_index = 0, y = 0; y < yres; y++)
    {
        for (         x = 0; x < xres; x++, c_index++)
        {
            // if the cell has data
            if (src.data[c_index] != 0)
            {
                // neighbours
                n_index = c_index - xres;
                n_label = (y == 0) ? 0 : dst.data[n_index];
                w_index = c_index - 1;
                w_label = (x == 0) ? 0 : dst.data[w_index];
                // if it has a labelled north.
                if (n_label)
                {
                    // and a labelled west.
                    if (w_label)
                    {
                        // and they're different.
                        if (n_label != w_label)
                        {
                            // mark the regions as equivalent (unless they've already been marked).
                            bool regions_already_equivalent = false;
                            for (i = 0; i < (int) regions_equivalence.size(); i++)
                            {
                                regions_already_equivalent |= (regions_equivalence[i].first == n_label && regions_equivalence[i].second == w_label) ||
                                                              (regions_equivalence[i].first == w_label && regions_equivalence[i].second == n_label);
                            }
                            if (!regions_already_equivalent)
                            {
                                regions_equivalence.push_back(std::pair<uint32_t, uint32_t>(MIN(n_label, w_label), MAX(n_label, w_label)));
                            }
                        }
                    }
                    // give it the north label.
                    region_label = (uint8_t) n_label;
                }
                else if (w_label)
                {
                    // otherwise mark it with the west label.
                    region_label = (uint8_t) w_label;
                }
                else
                {
                    // otherwise make it a new region.
                    region_label = (uint8_t) ++region_count;
                }
            }
            else
            {
                // no region for you.
                region_label = 0;
            }
            // save it.
            dst.data[c_index] = region_label;
        }
    }
    //printf("finished first pass\n");
    
    // merge equivalent regions down. we also compress the region labels back to a contiguous range, because
    // equivalent regions will have used extras, GOD DAMN IT MEG.
    if (regions_equivalence.size() > 0)
    {
        uint32_t j;
        uint32_t new_region_count;
        uint32_t duplicate_index;
        std::vector<uint32_t> region_mapping;
        region_mapping.push_back(0);

        // go through all the old regions
        for (i = 1, new_region_count = 1; i <= region_count; i++)
        {
            // check if the region currently being considered is equivalent to any other, lower numbered, regions.
            for (j = 0, duplicate_index = 0; j < regions_equivalence.size(); j++)
            {
                if (regions_equivalence[j].second == i)
                {
                    duplicate_index = regions_equivalence[j].first;
                    break;
                }
            }

            // if it is a duplicate region, set it to the old duplicate and don't update the new_region_count;
            if (duplicate_index != 0)
            {
                if (region_mapping.size() <= duplicate_index)
                    printf("uh oh... region_mapping.size()=%d, duplicate_index=%d\n", region_mapping.size(), duplicate_index);
                region_mapping.push_back(region_mapping[duplicate_index]);
            }
            else
                region_mapping.push_back(new_region_count++);
        }
        // apply the mapping
        for (i = 0; i < src.size().area(); i++)
            dst.data[i] = (uint8_t) region_mapping[dst.data[i]];
        //printf("finished second pass\n");
        return new_region_count;
    }
    else
    {
        //printf("no second pass necessary\n");
        return region_count+1;
    }
}


uint32_t* makey_the_hist (cv::Mat& frame)
{
    uint32_t* r = (uint32_t*) calloc(256, sizeof(uint32_t));
    uint32_t lim = frame.size().area();
    for (uint32_t i = 0; i < lim; i++)
        r[frame.data[i]]++;
    return r;
}


int suppressNoise (cv::Mat& frame)
{
    uint32_t v;
    uint32_t* hist;
    int reductions = 0;
    uint8_t i;
    int j;

    hist = makey_the_hist(frame);
    for (i = 1; i < 256; i++)
    {
        v = hist[i];
        if (v == 0)
            break;

        if (v < MARKER_EXTRACTION_SIZE_THRESHOLD)
        {
            reductions++;
            for (j = 0; j < frame.size().area(); j++)
                if (frame.data[j] == i)
                    frame.data[j] = 0;
        }
    }
    free(hist);

    return reductions;
}


void extractCorners (cv::Mat& frame, cv::Point_<int>* p)
{
    int x=0, y=0;
    int xres  = frame.cols, yres  = frame.rows;
    int hxres = xres / 2,   hyres = yres / 2;
    cv::Point_<int> candidate1,       candidate2;
    float           candidate1_dist2, candidate2_dist2;
    // add the images corner points
    cv::Point_<int> frame_tl(0,      0);
    cv::Point_<int> frame_tr(xres-1, 0);
    cv::Point_<int> frame_bl(0,      yres-1);
    cv::Point_<int> frame_br(xres-1, yres-1);


    // find top left corner
    // find leftmost
    for (x = 0; x < hxres; x++)
        for (y = 0; y < hyres; y++)
            if (frame.data[(y*xres) + x])
                goto tl_lm_corner;
tl_lm_corner: candidate1 = cv::Point_<int>(x, y);
    // find topmost
    for (y = 0; y < hyres; y++)
        for (x = 0; x < hxres; x++)
            if (frame.data[(y*xres) + x])
                goto tl_tm_corner;
tl_tm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_tl);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_tl);
    p[0] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
    

    // find bottom left corner
    // find leftmost
    for (x = 0; x < hxres; x++)
        for (y = yres-1; y >= hyres; y--)
            if (frame.data[(y*xres) + x])
                goto bl_lm_corner;
bl_lm_corner: candidate1 = cv::Point_<int>(x, y);
    // find bottommost
    for (y = yres-1; y >= hyres; y--)
        for (x = 0; x < hxres; x++)
            if (frame.data[(y*xres) + x])
                goto bl_bm_corner;
bl_bm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_bl);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_bl);
    p[1] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
    
    
    // find top right corner
    // find rightmost
    for (x = xres-1; x >= hxres; x--)
        for (y = 0; y < hyres; y++)
            if (frame.data[(y*xres) + x])
                goto tr_rm_corner;
tr_rm_corner: candidate1 = cv::Point_<int>(x, y);
    // find topmost
    for (y = 0; y < hyres; y++)
        for (x = xres-1; x >= hxres; x--)
            if (frame.data[(y*xres) + x])
                goto tr_tm_corner;
tr_tm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_tr);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_tr);
    p[2] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
    

    // find bottom right corner
    // find rightmost
    for (x = xres-1; x >= hxres; x--)
        for (y = yres-1; y >= hyres; y--)
            if (frame.data[(y*xres) + x])
                goto br_rm_corner;
br_rm_corner: candidate1 = cv::Point_<int>(x, y);
    // find bottommost
    for (y = yres-1; y >= hyres; y--)
        for (x = xres-1; x >= hxres; x--)
            if (frame.data[(y*xres) + x])
                goto br_bm_corner;
br_bm_corner: candidate2 = cv::Point_<int>(x, y);
    candidate1_dist2 = cv_euclidean_distance2(candidate1, frame_br);
    candidate2_dist2 = cv_euclidean_distance2(candidate2, frame_br);
    p[3] = (candidate1_dist2 < candidate2_dist2) ? candidate1 : candidate2;
}



void projectPoints (cv::Mat& depthFrame, cv::Point_<int>* p2D, cv::Point3_<float>* p3D, int count)
{
    static const int xres = depthFrame.cols, yres = depthFrame.rows;
    static const float projection_constant = (((PIXEL_SIZE * IMAGE_WIDTH * 2) / xres) / FOCAL_LENGTH);
    int i;
    
    for (i = 0; i < count; i++)
    {
        p3D[i].z = *((uint16_t*) (depthFrame.data + (((p2D[i].y * xres) + p2D[i].x)*2)));
        p3D[i].x = (p2D[i].x - (xres/2)) * p3D[i].z * projection_constant;
        p3D[i].y = ((yres/2) - p2D[i].y) * p3D[i].z * projection_constant;
    }
}


bool detectMarkerRegions (cv::Mat& grayscale, std::vector<cv::Rect>* regions)
{
    int i;
    std::vector<cv::Rect> internal_regions;
    /*cv::Mat gs_scaled(240, 320, CV_8UC1);
    kind_of_subsample(&gs_scaled, &grayscale, 2, 2, 0, 480);*/

    // Run the haar cascade to detect the marker.
    haar_cascade.detectMultiScale(grayscale, internal_regions, HAAR_SCALING, HAAR_NEIGHBOURS, CV_HAAR_SCALE_IMAGE);

    // If we haven't found any markers (most of the time), return.
    if (internal_regions.size() == 0)
        return false;

    // Replace the supplied list with the found marker(s).
    regions->clear();
    for (i = 0; i < (int) internal_regions.size(); i++)
    {
        regions->push_back(internal_regions[i]);
        /*(*regions)[i].x *= 2;
        (*regions)[i].y *= 2;
        (*regions)[i].width *= 2;
        (*regions)[i].height *= 2;*/
    }
    return true;
}


// NB: only needs color because we draw onto it.
bool extractMarkerFromRegions (cv::Mat& color, cv::Mat& grayscale, cv::Mat& depth, std::vector<cv::Rect>* regions, MarkerData* marker_data)
{
    int i, j;
    cv::Mat clipped_marker;
    cv::Point_<int>    marker_corners2[4];
    cv::Point3_<float> marker_corners3[4];
    float marker_width3d, marker_height3d, marker_depth3d;
    float marker_orientation3d, marker_unfoldedwidth, marker_unfoldedheight;
    cv::Point3_<float> marker_position3d;
    int regions_size = regions->size();

    // Okay, so we detected something - go through all the detected objects and analyse their likelihood of being our marker.
    for (i = 0; i < regions_size; i++)
    {
        // binarise the image around the given threshold.
        cv::threshold(grayscale((*regions)[i]), clipped_marker, MARKER_EXTRACTION_GS_THRESHOLD, 255, cv::THRESH_BINARY_INV);

        // 'cluster' the image into connected regions. remove those regions that are too small (and are thus probably noise).
        cv::Mat region_marker(clipped_marker.size(), CV_8UC1);
        //int region_count = connected_component_labelling(clipped_marker, region_marker);
        //int region_count_reduction = suppressNoise(region_marker);
        connected_component_labelling(clipped_marker, region_marker);
        suppressNoise(region_marker);
        //printf("detected %d regions, reduced to %d\n", region_count, region_count - region_count_reduction);

        // threshold this result. the regions will be numbers > 1, we don't care about what regions there are anymore, only
        // that there is a foreground region (the marker, minus noise) and a background region (not the marker).
        cv::threshold(region_marker, region_marker, 0, 255, cv::THRESH_BINARY);

        // from this thresholded image, extract the 4 corners that make up the marker (if this is the marker at all).
        extractCorners(region_marker, marker_corners2);
        // Convert these to global image co-ordinates.
        for (j = 0; j < 4; j++)
            marker_corners2[j] += (*regions)[i].tl();
        // convert these 4 2D points to 4 3D points.
        projectPoints(depth, marker_corners2, marker_corners3, 4);

        // extract their 3D info.
        marker_width3d  = ((marker_corners3[2].x - marker_corners3[0].x) + (marker_corners3[3].x - marker_corners3[1].x)) / 2.0f;
        marker_height3d = ((marker_corners3[0].y - marker_corners3[1].y) + (marker_corners3[2].y - marker_corners3[3].y)) / 2.0f;
        marker_depth3d  = ((marker_corners3[0].z - marker_corners3[2].z) + (marker_corners3[1].z - marker_corners3[3].z)) / 2.0f;
        marker_position3d = cv::Point3_<float>(((marker_corners3[0].x + marker_corners3[1].x + marker_corners3[2].x + marker_corners3[3].x) / 4.0f),
                                               ((marker_corners3[0].y + marker_corners3[1].y + marker_corners3[2].y + marker_corners3[3].y) / 4.0f),
                                               ((marker_corners3[0].z + marker_corners3[1].z + marker_corners3[2].z + marker_corners3[3].z) / 4.0f) );
        marker_orientation3d  = (float) (-atan(marker_depth3d / marker_width3d) * (180.0 / PI));
        marker_unfoldedwidth  = sqrt(marker_width3d*marker_width3d   + marker_depth3d*marker_depth3d);
        marker_unfoldedheight = sqrt(marker_height3d*marker_height3d + marker_depth3d*marker_depth3d);

        // check if the thing we're looking at could actually be a marker
        if ((marker_unfoldedwidth  > (MARKER_WIDTH  - MARKER_EPSILON)) && (marker_unfoldedwidth  < (MARKER_WIDTH  + MARKER_EPSILON)) &&
            (marker_unfoldedheight > (MARKER_HEIGHT - MARKER_EPSILON)) && (marker_unfoldedheight < (MARKER_HEIGHT + MARKER_EPSILON))   )
        {
            //printf("  MARKER DETECTED.  Position = (%.1f, %.1f, %.1f). Orientation = %.1f. Unfolded-size (wxh) = (%.1fx%.1f)\n", marker_position3d.x, marker_position3d.y, marker_position3d.z, marker_orientation3d, marker_unfoldedwidth, marker_unfoldedheight);
            
            // it is a marker! OMG! return it! (ignoring any subsequent marker objects... how could we discern them anyway?)
            marker_data->position    = marker_position3d;
            marker_data->orientation = marker_orientation3d;

            // also draw it while we can
            cv::line(color, marker_corners2[0], marker_corners2[1], cv::Scalar(255,0,255), 2, 8, 0);
            cv::line(color, marker_corners2[1], marker_corners2[3], cv::Scalar(255,0,255), 2, 8, 0);
            cv::line(color, marker_corners2[3], marker_corners2[2], cv::Scalar(0,255,255), 2, 8, 0);
            cv::line(color, marker_corners2[2], marker_corners2[0], cv::Scalar(0,255,255), 2, 8, 0);

            return true;
        }
        else
        {
            //printf("  MARKER LIKE OBJECT IGNORED 2d: (%.1fx%.1f).\n", marker_unfoldedwidth, marker_unfoldedheight);
            regions->erase(regions->begin() + i);
            regions_size--;
            i--;
        }
    }
    return false;
}


std::vector<std::string>& split(const std::string& s, char delim, std::vector<std::string>& elems)
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim))
        elems.push_back(item);
    return elems;
}

std::vector<std::string> split(const std::string& s, char delim)
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void testCascade (void)
{
    // load the cascade
    if (!haar_cascade.load(cascade_name))
    {
        printf("Failed to load haar cascade GOOD GOD, PLEASE NO.\n");
        return;
    }

    // load the file list
    ifstream input_file;
    string   line_read;
    string   folder_absolute_address = "C:\\Users\\Jon\\Dropbox\\project_haar\\temp\\testing_marker2\\";
    input_file.open(folder_absolute_address + "info.txt");

    // read the file list
    int total_file_count = 0;
    int correct_classification_count = 0;
    int incorrect_classification_count = 0;
    while (input_file.good())
    {
        std::getline(input_file, line_read);
        std::vector<std::string> split_line = split(line_read, ' ');
        if (split_line.size() < 6)
            break;
        total_file_count++;
        cout << "processing file " << total_file_count << endl;
        // open the currently listed file.
        cv::Mat image_gs = cv::imread(folder_absolute_address + split_line[0], CV_LOAD_IMAGE_GRAYSCALE);
        cv::equalizeHist(image_gs, image_gs);
        cv::Rect ground_truth(atoi(split_line[2].c_str()), atoi(split_line[3].c_str()), atoi(split_line[4].c_str()), atoi(split_line[5].c_str()));
        cv::Rect inflated_ground_truth(ground_truth.x - (ground_truth.width  / 4) - 20,
                                       ground_truth.y - (ground_truth.height / 4) - 20,
                                       (int) ((ground_truth.width  * 1.5) + 40),
                                       (int) ((ground_truth.height * 1.5) + 40));
        
        // test it
        std::vector<cv::Rect> detected_objects;
        haar_cascade.detectMultiScale(image_gs, detected_objects, HAAR_SCALING, HAAR_NEIGHBOURS, CV_HAAR_SCALE_IMAGE);
        
        // check it
        bool correctly_identified = false;
        for (int i = 0; i < detected_objects.size(); i++)
        {
            if (!correctly_identified && inflated_ground_truth.contains(detected_objects[i].tl()) && inflated_ground_truth.contains(detected_objects[i].br()))
            {
                correctly_identified = true;
                correct_classification_count++;
            }
            else
            {
                incorrect_classification_count++;
            }
        }
    }

    printf("woo we're done!\n");
    printf("\n---- PARAMETERS ----\n");
    printf("SF = %.1f, #N = %d\n", HAAR_SCALING, HAAR_NEIGHBOURS);
    printf("\n---- RESULTS ----\n");
    printf("made a total of %d classifications\n", correct_classification_count + incorrect_classification_count);
    printf("%d / %d correctly classified\n", correct_classification_count, total_file_count);
    printf("%d incorrectly classified\n",  incorrect_classification_count);

    input_file.close();
}




















struct AdjNode
{
    uint32_t id;
    float    prox;

    AdjNode (uint32_t id_init, float prox_init) : id(id_init), prox(prox_init)
    {
    }
};
struct Node
{
    std::vector<AdjNode> adj;
    float  dist;
    Point2 p;

    Node (uint16_t x, uint16_t y) : p(x, y)
    {
    }
    Node (Point2 p_init) : p(p_init)
    {
    }
};
typedef std::vector<Node> Graph;
Graph G;
struct comp
{
    bool operator() (const int& a, const int& b)
    {
        return G[a].dist > G[b].dist;
    }
};

float euclidean_distance (Point2 p1, Point2 p2)
{
    float dx = (float) p1.x - p2.x;
    float dy = (float) p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

// pushes a new point onto the graph and connects (bi-directionally) to all nodes in the list given.
void graph_insert (Point2 p, std::vector<uint32_t> connection_ids)
{
    uint32_t i, graph_size = G.size();
    std::vector<float> connection_distances;
    for (i = 0; i < connection_ids.size(); i++)
        connection_distances.push_back(euclidean_distance(p, G[connection_ids[i]].p));

    // make the new node and add the adjacent nodes to it.
    Node n_new(p);
    for (i = 0; i < connection_ids.size(); i++)
        n_new.adj.push_back(AdjNode(connection_ids[i], connection_distances[i]));

    // add the new node to the adjacent nodes.
    AdjNode a_old(graph_size, UINT32_MAX);
    for (i = 0; i < connection_ids.size(); i++)
        G[connection_ids[i]].adj.push_back(AdjNode(graph_size, connection_distances[i]));

    // add the new node to the graph.
    G.push_back(n_new);
}

// pushes a new point to the end of the graph, linking it to the last node (path-like).
void graph_push_back (Point2 p)
{
    int graph_size = G.size();
    Node n_new(p);

    if (graph_size > 0)
    {
        Node& n_old = G[graph_size-1];
        float node_distance = euclidean_distance(n_old.p, p);
    
        // make the new node and add the previous node to it's adjacency.
        AdjNode a_new(graph_size-1, node_distance);
        n_new.adj.push_back(a_new);

        // add the new node to the previous node's adjacency.
        AdjNode a_old(graph_size,   node_distance);
        n_old.adj.push_back(a_old);
    }

    // add the new node to the graph.
    G.push_back(n_new);
}

void debug_build_graph (void)
{
    // A (0)
    graph_push_back(Point2(0, 0));

    std::vector<uint32_t> adjacent_ids;

    // B (1)
    adjacent_ids.clear();
    adjacent_ids.push_back(0);
    graph_insert(Point2(0, 10), adjacent_ids);

    // C (2)
    adjacent_ids.clear();
    adjacent_ids.push_back(0);
    adjacent_ids.push_back(1);
    graph_insert(Point2(5, 5), adjacent_ids);

    // D (3)
    adjacent_ids.clear();
    adjacent_ids.push_back(1);
//    adjacent_ids.push_back(2);
    graph_insert(Point2(10, 10), adjacent_ids);
}

float graph_dijkstra (uint32_t start, uint32_t end)
{
    std::priority_queue<uint32_t, std::vector<uint32_t>, comp> Q;
    uint32_t u, v, i;
    float alt;
        
    // initialise
    for (i = 0; i < G.size(); i++)
        G[i].dist = INFINITY;
    G[start].dist = 0;
    Q.push(start);
    
    while (Q.size() != 0)
    {
        // grab the minimum element (and check it's not whack).
        u = Q.top();
        Q.pop();
//        printf("visiting %d. G[%d].dist=%.2f\n", u, u, G[u].dist);
        if (G[u].dist == INFINITY)
            break;

        // relax the dijkstra
        for (i = 0; i < G[u].adj.size(); i++)
        {
            v   = G[u].adj[i].id;                   // the ith neighbour of u
            alt = G[u].dist + G[u].adj[i].prox;     // the total distance to v
//            printf("  relaxing. v=%d, alt=%.2f+%.2f, G[%d].dist=%.2f\n", v, G[u].dist, G[u].adj[i].prox, v, G[v].dist);
            if (alt < G[v].dist)
            {
                G[v].dist = alt;
                Q.push(v);
//                printf("  set G[%d].dist=%.2f\n", v, G[v].dist);
            }
        }
    }
    return G[end].dist;
}






void opencv_camera_test (void)
{
    CvCapture* capture;
    cv::Mat    frame;
    capture = cvCaptureFromCAM(-1);

    if (capture)
    {
        while (true)
        {
            frame = cvQueryFrame(capture);
            if (frame.empty())
                break;
            cv::imshow("mywindow", frame);
            printf("frame = %d x %d\n", frame.cols, frame.rows);
            if (cv::waitKey(10) > 0)
                break;
        }
    }
   // Release the capture device housekeeping
   cvReleaseCapture( &capture );
   cvDestroyWindow( "mywindow" );
}



//#define DELETE_THIS_NAUGHTINESS





XnBool fileExists(const char *fn)
{
    XnBool exists;
    xnOSDoesFileExist(fn, &exists);
    return exists;
}


int main (int argc, char* argv[])
{
    // naughtiest hack
    debug_build_graph();
    float d = graph_dijkstra(0, 3);
    printf("d=%.2f\n", d);
    //return 0;


    // naughtier hack
    //opencv_camera_test();
    //return 0;


    // still naughty
    //testCascade();
    //return 0;


    // naughty hack
    //opencv_depthy_tests();
    //return 0;


#ifdef DELETE_THIS_NAUGHTINESS
    CvCapture* capture = cvCaptureFromCAM(-1);
#endif


    // normally starts here... ssshhhhhhhhhhhh nobody will know.
    const char *fn = NULL;
    if    (fileExists(SAMPLE_XML_PATH))
    {
        fn = SAMPLE_XML_PATH;
    }
    else if (fileExists(SAMPLE_XML_PATH_LOCAL))
    {
        fn = SAMPLE_XML_PATH_LOCAL;
    }
    else
    {
        printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
        return XN_STATUS_ERROR;
    }
    printf("Reading config from: '%s'\n", fn);
    nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

    if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    }
    else if (nRetVal != XN_STATUS_OK)
    {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }

    nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
    CHECK_RC(nRetVal, "Find depth generator");
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_IMAGE, color);
    CHECK_RC(nRetVal, "Find color generator");
    if(depth.IsCapabilitySupported("AlternativeViewPoint"))
    {
        nRetVal = depth.GetAlternativeViewPointCap().SetViewPoint(color);
        CHECK_RC(nRetVal, "SetViewPoint for depth generator");
    }

    nRetVal = xnFPSInit(&xnFPS, 180);
    CHECK_RC(nRetVal, "FPS Init");

    double pixel_size;
    XnUInt64 focal_length;
    depth.GetRealProperty("ZPPS", pixel_size);
    depth.GetIntProperty( "ZPD",  focal_length);
    printf("ZPPS=%f, ZPD=%d\n", pixel_size, focal_length);

    // Load haar cascade
    if (!haar_cascade.load(cascade_name))
    {
        printf("Failed to load haar cascade GOOD GOD, PLEASE NO.\n");
        return 1;
    }
    
    bool marker_seen=false, marker_found=false;
    std::vector<cv::Rect> regions;
    MarkerData marker_data;
    uint32_t marker_counter=0;
    printf("Running initialisation frames...\n");
    std::deque<bool> marker_found_queue;
#define TEST_LENGTH 50
    bool test_running = false;
    bool exit_condition = false;
    int  test_length  = 0;
    int  test_counter = 0;
    int  test_reset_counter = 0;
    for (int i = 0; i < TEST_LENGTH; i++)
        marker_found_queue.push_front(false);
    //for (int i = 0; i < INIT_FRAMES; i++)
    while (!exit_condition)
    {
        nRetVal = context.WaitAndUpdateAll();
        if (nRetVal != XN_STATUS_OK)
        {
            printf("  UpdateData failed (but still continuing): %s\n", xnGetStatusString(nRetVal));
            continue;
        }
        xnFPSMarkFrame(&xnFPS);

        depth.GetMetaData(depthMD);
        color.GetMetaData(colorMD);

        cv::Mat depthDataMat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);
#ifndef DELETE_THIS_NAUGHTINESS
        cv::Mat colorDataMat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
#endif
        memcpy(depthDataMat.data, depthMD.Data(), IMAGE_HEIGHT * IMAGE_WIDTH * 2);
#ifndef DELETE_THIS_NAUGHTINESS
        memcpy(colorDataMat.data, colorMD.Data(), IMAGE_HEIGHT * IMAGE_WIDTH * 3);
#else
        cv::Mat colorDataMat = cvQueryFrame(capture);
#endif
        cv::Mat gsDataMat;

        if (!colorDataMat.empty())
        {
            // First make a grayscale map of the image. Equalising the histogram has the effect of removing
            // a lot of the effects of differing light levels and increasing contrast.
            cv::cvtColor(colorDataMat, colorDataMat, CV_RGB2BGR);
            cv::cvtColor(colorDataMat, gsDataMat,    CV_BGR2GRAY);
            cv::equalizeHist(gsDataMat, gsDataMat);

            
            // detect markers
            marker_seen = detectMarkerRegions(gsDataMat, &regions);
            if (regions.size() > 0)
                marker_found = extractMarkerFromRegions(colorDataMat, gsDataMat, depthDataMat, &regions, &marker_data);
            else
                marker_found = false;
            if (test_reset_counter == 0)
            {
                marker_found_queue.pop_back();
                marker_found_queue.push_front(marker_found);
                marker_counter = 0;
                for (int i = 0; i < TEST_LENGTH; i++)
                    if (marker_found_queue[i])
                        marker_counter++;
                if (!test_running && marker_found)
                {
                    test_running = true;
                    test_length  = 0;
                    test_counter++;
                    printf("TEST %d: Depth = %.0f mm, Orientation = %.0f degrees", test_counter, marker_data.position.z, marker_data.orientation);
                    Beep(500, 600);
                }
                if (test_running)
                {
                    if (test_length == TEST_LENGTH)
                    {
                        test_running = false;
                        printf(" - found %d/%d markers @ %.1f FPS\n", marker_counter, TEST_LENGTH, xnFPSCalc(&xnFPS));
                        //exit_condition |= true;
                        test_reset_counter = 50;
                        Beep(500, 600);
                    }
                    else
                    {
                        test_length++;
                    }
                }
            }
            else
            {
                if (--test_reset_counter == 0)
                {
                    //printf("reset\n");
                    Beep(1000, 600);
                }
            }

            cv::imshow("OMG", colorDataMat);
            //cv::imshow("deep", (depthDataMat * 8));
        }
        else
        {
            printf("Whoops, we appear to have failed to capture a frame... lets pretend this didn't happen and move on\n");
            continue;
        }

        exit_condition |= (cv::waitKey(10) > 10);
        
        //printf("Frame %d, FPS: %f\n", depthMD.FrameID(), xnFPSCalc(&xnFPS), marker_counter);
    }
    //return 0;


    



    /*
    CvCapture* capture;
    cv::Mat    frame;
    cv::Mat    frame_gray;
    cv::Rect   marker_window;
    bool       marker_seen;
    uint8_t    marker_cooldown = 0;

    //-- 1. Load the cascade
    if (!haar_cascade.load(cascade_name))
    {
        printf("--(!)Error loading\n");
        return;
    };

    //-- 2. Read the video stream
    capture = cvCaptureFromCAM(-1);
    if (capture)
    {
        while (true)
        {
            frame = cvQueryFrame(capture);

            //-- 3. Apply the classifier to the frame
            if (!frame.empty())
            {
                cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
                marker_seen = detectMarker(&frame, &frame_gray, &marker_window);
                if (marker_seen)
                {
                    marker_cooldown = 5;
                    displayMarker(&frame, &frame_gray, &marker_window);
                }
                else if (marker_cooldown > 0)
                {
                    marker_cooldown--;
                    displayMarker(&frame, &frame_gray, &marker_window);
                }
                cv::imshow("window_OMG", frame);
            }
            else
            {
                printf(" --(!) No captured frame -- Break!");
                break;
            }

            int c = cv::waitKey(10);
            if (c > 0)
                break;
        }
    }
    return;*/






    /*
    printf("doing something?\n");
    glutInit(&argc, argv);
    glutCreateWindow(700, 100, MATPLOT_WINDOW_WIDTH, MATPLOT_WINDOW_HEIGHT);    // originally 1200x1000
    glutDisplayFunc( display );
    glutReshapeFunc( reshape );
    glutIdleFunc( idle );
    glutMotionFunc( motion );
    glutMouseFunc( mouse );
    glutPassiveMotionFunc(passive);    
    glutKeyboardFunc( keyboard );        
    printf("hi\n");
    glutMainLoop();
    printf("done>?!?!!!?\n");
    */




    /*cv::namedWindow(STREAM_WINDOW_NAME, CV_WINDOW_AUTOSIZE);
    
    IplImage* img = cvLoadImage("cat.jpg");
    cvNamedWindow("ImageUno", 1);
    cvShowImage("ImageUno", img);
    cvWaitKey();
    cvDestroyWindow("ImageUno");
    cvReleaseImage(&img);
    cvDestroyWindow(STREAM_WINDOW_NAME);*/

    /*
    FILE* f;
    const char* file_path_base = "D:\\Users\\Jon\\Dropbox\\frame_";
    const char* file_path_ext  = ".dat";
    const int   file_path_size = strlen(file_path_base) + 2 + strlen(file_path_ext) + 1;
    printf("%d\n", file_path_size);
    char*       file_path = (char*) malloc(sizeof(char) * file_path_size);

    printf("Running capture frames...\n");
    for (int i = 0; i < CAPTURE_FRAMES; i++)
    {
        nRetVal = context.WaitOneUpdateAll(depth);
        if (nRetVal != XN_STATUS_OK)
        {
            printf("  UpdateData failed (but still continuing): %s\n", xnGetStatusString(nRetVal));
            i--;
            continue;
        }
        xnFPSMarkFrame(&xnFPS);

        depth.GetMetaData(depthMD);
        const XnDepthPixel* depthData = depthMD.Data();

        printf("  Writing Frame %d, FPS: %f...", depthMD.FrameID(), xnFPSCalc(&xnFPS));
        fflush(stdout);
        
        // write to file
        sprintf(file_path, "%s%d%s", file_path_base, i, file_path_ext);
        f = fopen(file_path, "wb");
        fwrite(depthData, sizeof(XnDepthPixel), depthMD.XRes() * depthMD.YRes(), f);
        fclose(f);
        printf(" done.\n");
    }*/
    
    /*cv::Mat outputDepthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    // update + grab our data
    nRetVal = context.WaitOneUpdateAll(depth);
    if (nRetVal != XN_STATUS_OK)
        printf("  UpdateData failed on image write... this could be ugly: %s\n", xnGetStatusString(nRetVal));
    const XnDepthPixel* depthData = depthMD.Data();

    // create + save the image
    createColourDepthImage(&outputDepthImage, (const uint16_t*) depthData);
    cv::imwrite("D:\\Users\\Jon\\Dropbox\\output_frame.png", outputDepthImage);
    */

    depth.Release();
    color.Release();
    scriptNode.Release();
    context.Release();

    return 0;
}
