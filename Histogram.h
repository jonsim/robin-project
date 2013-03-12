/**
 * @file    Histogram.h
 * @author  Jonathan Simmonds
 * @brief     A class to build and store a histogram from a dataset.
 */
#ifndef HISTOGRAM_H
#define HISTOGRAM_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdlib.h>




/*-------------------- DEFINES  --------------------*/
#define HIST_MAX_VALUE              5000 // mm




/*-------------- CLASS DEFINITIONS --------------*/
/**
 * @brief   A class to build and store a histogram from a dataset. It is possible to rebuild the
 *          histogram on the fly using the same amount of memory. This is useful for changing data
 *          from the same population (e.g. subsequent frames in a video sequence).
 */
class Histogram
{
public:
    /// @brief  Constructor. The maximum value the histogram is able to take is given by the HIST_MAX_VALUE
    ///         define. Use a different constructor if this needs to be dynamic/different. The histogram is
    ///         initialised to zeros.
    Histogram (void) : mMaxValue(HIST_MAX_VALUE)
    {
        mHist = (uint32_t*) calloc(mMaxValue+1, sizeof(uint32_t));
    }
    
    
    /// @brief  Constructor.
    /// @param  maximum_value   The maximum value the histogram is able to take. The histogram is
    ///                         initialised to zeros. If data needs to be initially loaded in, use a
    ///                         different constructor.
    Histogram (const uint16_t maximum_value) : mMaxValue(maximum_value)
    {
        mHist = (uint32_t*) calloc(mMaxValue+1, sizeof(uint32_t));
    }
    
    
    /// @brief  Constructor.
    /// @param  data            The data with which to initialise the histogram.
    /// @param  data_count      The number of items to read from the data array.
    /// @param  maximum_value   The maximum value the histogram is able to take.
    Histogram (const uint16_t* data, const uint32_t data_count, const uint16_t maximum_value) : mMaxValue(maximum_value)
    {
        uint32_t one_step_index;

        mHist = (uint32_t*) calloc(mMaxValue+1, sizeof(uint32_t));
        for (one_step_index = 0; one_step_index < data_count; one_step_index++)
            mHist[MIN(data[one_step_index], mMaxValue)]++;
    }
    
    
    /// @brief  Deconstructor.
    ~Histogram (void)
    {
        free(mHist);
    }
    
    
    /// @brief  Zeros the histogram before rebuilding it from the data given. This uses the same max
    ///         value as before. If a different maximum value is needed a new Histogram must be made.
    /// @param  data        The data with which to rebuild the histogram.
    /// @param  data_count  The number of items to read from the data array.
    void rebuild (const uint16_t* data, const uint32_t data_count)
    {
        uint32_t one_step_index;

        memset(mHist, 0, (mMaxValue+1) * sizeof(uint32_t));
        for (one_step_index = 0; one_step_index < data_count; one_step_index++)
            mHist[MIN(data[one_step_index], mMaxValue)]++;
    }
    
    void rebuildLeftHalf (const uint16_t* data, const uint16_t xres, const uint16_t yres)
    {
        uint16_t x, y;
        uint16_t half_xres = xres/2;
        uint32_t one_step_index;
        
        memset(mHist, 0, (mMaxValue+1) * sizeof(uint32_t));
        for (one_step_index = 0, y = 0; y < yres;      y++, one_step_index+=half_xres)
            for (         x = 0;        x < half_xres; x++, one_step_index++)
                mHist[MIN(data[one_step_index], mMaxValue)]++;
    }
    
    void rebuildRightHalf (const uint16_t* data, const uint16_t xres, const uint16_t yres)
    {
        uint16_t x, y;
        uint16_t half_xres = xres/2;
        uint32_t one_step_index;
        
        memset(mHist, 0, (mMaxValue+1) * sizeof(uint32_t));
        for (one_step_index = half_xres, y = 0; y < yres; y++, one_step_index+=half_xres)
            for (         x = half_xres;        x < xres; x++, one_step_index++)
                mHist[MIN(data[one_step_index], mMaxValue)]++;
    }
    
    
    /// @brief  Retrieves the count (number of occurances) of a given value from the histogram.
    /// @brief  value   The value to retrieve.
    /// @return         The number of occurances of value in the histogram.
    inline uint32_t get (const uint16_t value) const
    {
        return mHist[MIN(value, mMaxValue)];
    }
    
    
    /// @brief  Retrieves the count (number of occurances) of the values in a given inclusive range.
    /// @param  starting_value  The value at the start of the range.
    /// @param  ending_value    The value at the end of the range.
    /// @return The number of occurances of all values in the range.
    inline uint32_t getRange (uint16_t starting_value, uint16_t ending_value) const
    {
        uint16_t i;
        uint32_t accumulator = 0;

        ending_value = MIN(ending_value, mMaxValue);
        for (i = starting_value; i <= ending_value; i++)
            accumulator += mHist[i];
        return accumulator;
    }
    
    inline void getRangeAndStore (uint16_t starting_value, uint16_t ending_value, const uint8_t store_position)
    {
        uint32_t r = getRange(starting_value, ending_value);
        if (store_position == 1)
            mRange1_store = r;
        else if (store_position == 2)
            mRange2_store = r;
    }
    
    inline uint32_t retrieveRangeFromStore (const uint8_t store_position) const
    {
        if (store_position == 1)
            return mRange1_store;
        if (store_position == 2)
            return mRange2_store;
        return 0;
    }
    
    
private:
    uint32_t  mRange1_store;  ///< A storage space for a range output.
    uint32_t  mRange2_store;  ///< A storage space for a range output.
    uint16_t  mMaxValue;      ///< The maximum value the histogram can represent.
    uint32_t* mHist;          ///< The array in which the histogram data is stored.
};

#endif
