/**
 * @file    FrameBuffer.h
 * @author  Jonathan Simmonds
 * @brief     A class to hold frames for a given amount of time and perform actions on them.
 */
#ifndef FRAMEBUFFER_H
#define FRAMEBUFFER_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "CircularBuffer.h"
#include "BinaryMarker.h"
#include "Histogram.h"




/*-------------------- DEFINES --------------------*/
#define HIST_NEAR_RANGE_START        400 // mm
#define HIST_NEAR_RANGE_END          600 // mm
#define DIFFERENCE_THRESHOLD          30 // mm
#define HIST_STATIC_PANIC_THRESHOLD  180000 // this many readings in the near range will cause instant panic
#define HIST_DYNAMIC_PANIC_THRESHOLD  13000 // this many readings suddenly transitioning out of the near range will cause panic




/*-------------- CLASS DEFINITIONS --------------*/
/**
 * @brief   A class to hold frames for a given amount of time and perform actions on them.
 *
 *          The frame retention (number of frames which are stored at once) can be specified.
 *          A focus has been made on efficiency of the data structures used to store the data to
 *          make this as easy on memory as possible. Importantly only linear memory is used, O(nm)
 *          where n is the number of pixels per frame and m is the number of frames stored. All
 *          actions on the buffer (insert/retrieve etc.) should take linear time O(n) (as above).
 */
class FrameBuffer
{
public:
    /// @brief  Constructor.
    /// @param  xres    The x-resolution of the input data. The resolution of the output data is given
    ///                 by the input x-resolution divided by the sampling factor. It is recommended
    ///                 therefore that the sampling_factor is a factor of the x-resolution.
    /// @param  yres    The y-resolution of the input data. The resolution of the output data is given
    ///                 by the input y-resolution divided by the sampling factor. It is recommended
    ///                 therefore that the sampling_factor is a factor of the y-resolution.
    /// @param  sampling_factor   The sampling factor to use to reduce the input data for storage in
    ///                 the frame buffer. The data will be sampled every sampling_factor steps in each
    ///                 dimension. Therefore 1/sampling_factor-th of the x-data and y-data will be
    ///                 used, or 1/(sampling_factor^2)-th of the total data. This can be 1 to use all
    ///                 the data in the input array.
    /// @param  frame_retention   The number of previous frames (added with the insert() function) to
    ////                store. Frames get discarded when they pass this limit.
    FrameBuffer  (const uint16_t xres, const uint16_t yres, const uint16_t sampling_factor, const uint8_t frame_retention) :
      output_xres(xres / sampling_factor),
      output_yres(yres / sampling_factor),
      output_frame_res(output_xres * output_yres),
      subtractionBuffer(output_frame_res),
      input_xres(xres),
      input_yres(yres),
      input_frame_res(input_xres * input_yres),
      resampling_factor(sampling_factor),
      pixel_size((PIXEL_SIZE * IMAGE_WIDTH * 2) / output_xres),
      projection_constant(pixel_size / FOCAL_LENGTH),
      histogramNearRangeCount(0),
      histogramErrorRangeCount(0),
      retention(frame_retention),
      lead_in_count(frame_retention),
      dataBuffer(frame_retention),   // TODO not because of the way I use the buffer the extra space which is supposed to be free isn't.
                                     // this is not a problem but means a frame_retention of < 2 is not possible. Also the pre-load would break.
      histogramBuffer(frame_retention) // same problem as above
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
    
    
    /// @brief  Deconstructor.
    ~FrameBuffer (void)
    {
        free(rawDataBuffer);
    }
    
    
    /// @brief  = operator. This is only overridden to prevent warnings being generated. Due to the use of
    ///         constant class members this is not possible. Thus use of the = operator is forbidden.
    FrameBuffer& operator= (const FrameBuffer& other)
    {
        if (this != &other)
        {
            printf("FrameBuffer Assignment Operator is disallowed.\n");
            exit(EXIT_FAILURE);
        }
        return *this;
    }
    
    
    /// @brief  Inserts the data given into the frame buffer. This data must be of the x/y-resolutions
    ///         given in the constructor. Sub-sampling is done in accordance with the sampling_factor
    ///         supplied in the constructor.
    ///         frame_retention insertions must be made before ANY data can be retrieved.
    /// @param  data    The data to insert into the frame buffer.
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
    
    
    /// @brief  Extracts the data associated with a given frame.
    /// @param  frame_number    The number of the frame whose data is to be extracted. If this is
    ///                         invalid (i.e. out of bounds) NULL will be returned. A frame_number
    ///                         of 0 is the most recent frame stored.
    /// @return A pointer to the head of the data array associated with the given frame number. This
    ///         is the FrameBuffer's own copy so any alterations made to this array will be reflected
    ///         in the FrameBuffer. This data array has the dimensions given by the input x/y-resolution
    ///         divided by the sampling_factor, given in the constructor.
    ///         NULL can be returned if frame number is not in the range 0 <= frame_number < frame_retention.
    ///         NULL can also be returned if the FrameBuffer is not yet warmed up (i.e. it hasn't had
    ///         frame_retention insertions. The caller MUST check for NULL returns.
    uint16_t* retrieve (const uint8_t frame_number) const
    {
        if ((frame_number < retention) && (lead_in_count == 0))
            return *(dataBuffer.get(frame_number));
        else
            return NULL;
    }
    
    
    /// @brief  Fills the subtraction buffer with the result of the thresholded difference between
    ///         the data in the two frames given.
    ///         The subtraction buffer is a BinaryMarker (see BinaryMarker.h) which spans the length
    ///         of a single internal frame data array and for each pixel contains either a 0, if the
    ///         difference between the pixel in the two frames falls within the DIFFERENCE_THRESHOLD
    ///         preprocessor definition, or a 1 if the difference falls outside (i.e. difference > DIFFERENCE_THRESHOLD).
    /// @param  frame_number1   The index of the first frame (see the retrieve function for further
    ///                         information on this).
    /// @param  frame_number2   The index of the second frame (see the retrieve function for further
    ///                         information on this).
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
    
    
    /// @brief  As fillSubtractionBuffer but applies the provided transformation to the frame prior to
    ///         calculating the difference. This allows application of a motion model prior to the
    ///         generation of the subtractionBuffer. TODO DOES NOT YET WORK.
    /// @param  frame_number1   see fillSubtractionBuffer().
    /// @param  frame_number2   see fillSubtractionBuffer().
    /// @param  translation     The translation to apply to the pixel.
    /// @param  y_rotation      The rotation around the VERTICAL (y) axis to perform. The assumption
    ///                         is that the motion model will not need to be sophisticated enough to
    ///                         account for full 3-dimensional rotation (which would require quaternions etc).
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
    
    
#ifdef MATPLOT
    /// @brief  Projects all the pixels in a given frame to 3D space and fills in their x/y/z
    ///         co-ordinates in the provided vectors, in order of their appearence in the data array.
    ///         This function should only be used for 3D visualisation.
    /// @param  frame_number    The frame whose points will be projected.
    /// @param  x   The std::vector in which the x-coordinates will be put.
    /// @param  y   The std::vector in which the y-coordinates will be put.
    /// @param  z   The std::vector in which the z-coordinates will be put.
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


    /// @brief  Projects all the pixels in a given frame to 3D space and fills in their x/y/z
    ///         co-ordinates in the provided vectors, in order of their appearence in the data array.
    ///         This function should only be used for 3D visualisation.
    ///         This is masked by the subtraction buffer so that only pixels with a 1 in the
    ///         subtraction buffer are projected. The subtraction buffer must therefore be filled with
    ///         something meaningful by the fillSubtractionBuffer() function.
    /// @param  frame_number    The frame whose points will be projected.
    /// @param  x   The std::vector in which the x-coordinates will be put.
    /// @param  y   The std::vector in which the y-coordinates will be put.
    /// @param  z   The std::vector in which the z-coordinates will be put.
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
#endif
    
    
    /// @brief  Calculates the Euclidian-Distance^2 between two points (given as their 2D x/y pixel
    ///         co-ordinates) in 3D space. These points are projected in order for this to happen,
    ///         a task which could be made more efficient depending on the drivers/information available.
    /// @param  frame_number    The frame in which the distance is to be calculated.
    /// @param  p1              The first point.
    /// @param  p2              The second point.
    /// @return The Euclidian-Distance^2. It is important to note that this is the SQUARED DISTANCE,
    ///         the true distance can be found by square-rooting the answer, however for efficiency
    ///         this is not done in the function.
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

    
    /// @brief  As the euclidian_distance2() function, however it returns a boolean based on whether
    ///         or not the distance between two given points exceeds a given threshold.
    ///         Due to the structure of the data this is an easier question to answer than finding the
    ///         exact distance between the points.
    /// @param  frame_number    The frame in which the distance is to be calculated.
    /// @param  p1              The first point.
    /// @param  p2              The second point.
    /// @param  threshold2      The square of the threshold to compare the distance to.
    /// @return Returns TRUE if the distance between the two points is LESS THAN the threshold.
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

    
    const uint16_t output_xres;         ///< The x-resolution of the stored (or 'output') frames.
    const uint16_t output_yres;         ///< The y-resolution of the stored (or 'output') frames.
    const uint32_t output_frame_res;    ///< The number of pixels (x * y resolutions) of the stored (or 'output') frames.
    BinaryMarker subtractionBuffer;     ///< The subtraction buffer which is used to hold a threhsolded distance value between pixels.

private:
    const uint16_t input_xres;          ///< The x-resolution of the data passed into the buffer (or 'input' frames).
    const uint16_t input_yres;          ///< The y-resolution of the data passed into the buffer (or 'input' frames).
    const uint32_t input_frame_res;     ///< The number of pixels (x * y resolution) of the data passed into the buffer (or 'input' frames).
    const uint16_t resampling_factor;   ///< The number of pixels considered for sub-sampling from the 'input' to the 'output' frames.
    const float    pixel_size;          ///< The physical size of a pixel at the zero plane. Assumes square (cube) pixels.
    const float    projection_constant; ///< pixel_size / focal_length.

    uint32_t histogramNearRangeCount;   ///< The number of pixels in the near range of the histogram.
    uint32_t histogramErrorRangeCount;  ///< The number of pixels in the error range of the histogram.

    const uint8_t  retention;           ///< The frame retention.
    uint8_t        lead_in_count;       ///< The number of frames that still need to be inserted before the FrameBuffer can be accessed.
    CircularBuffer<uint16_t*> dataBuffer;   ///< The circular buffer which 'wraps' the frame data, managing its storage.
    uint16_t*      rawDataBuffer;       ///< The raw array used to store the frame data.
    CircularBuffer<Histogram*> histogramBuffer; ///< The circular buffer which 'wraps' the histogram data, managing its storage.
    Histogram*     rawHistogramBuffer;  ///< The raw array used to store the histogram data.

};

#endif
