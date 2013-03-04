/**
 * @file    BinaryMarker.h
 * @author  Jonathan Simmonds
 * @brief     A class to emulate an array of bits.
 */
#ifndef BINARYMARKER_H
#define BINARYMARKER_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdlib.h>




/*-------------- CLASS DEFINITIONS --------------*/
/**
 * @brief   A class to emulate an array of bits. This allows memory-efficient storage of binary
 *          information.
 *          
 *          It should be noted that this will not be as fast as a straight array of bytes where each
 *          byte encodes a single bit, as 5 arithmetic operations need to be made per set and 6 per 
 *          get, however it will be vastly superior on memory performance, especially for large
 *          amounts of data.
 */
class BinaryMarker
{
public:
    /// @brief  Constructor.
    /// @param  size  The length of the array (in bits).
    BinaryMarker (uint32_t size)
    {
        // Assign as many bytes as necessary to hold the number of bits. A small amount of wasted
        // space will have to be taken on the end of the array if the size is not a factor of 8.
        mBitSize  = size;
        mByteSize = (uint32_t) ceil(size / 8.0f);
        A = (uint8_t*) calloc(mByteSize, sizeof(uint8_t));
    }
    
    
    /// @brief  Deconstructor.
    ~BinaryMarker (void)
    {
        free(A);
    }
    
    
    /// @brief  Sets an element to the binary value specified.
    /// @param i      The index of the element to set.
    /// @param state  The state to set the element to (0 = 0, >0 = 1).
    void set (const uint32_t i, const uint8_t state)
    {
        // Bounds check.
        if (i > mBitSize)
            return;
        
        // Generate the required mask for extracting the data.
        uint32_t index = i >> 3;                    // i / 8
        uint8_t  pos   = (uint8_t) (i - (index*8)); // i % 8
        uint8_t  mask  = 1 << pos;
        
        // Apply the mask as necessitated by the state to either set the bit to 0 or 1.
        if (state)
            A[index] |= mask;   // set the bit to 1
        else
            A[index] &= !mask;  // set the bit to 0
    }
    
    
    /// @brief  Gets the state of an element.
    /// @param  i   The index of the element to get.
    /// @return The state of the element (0 or 1).
    uint8_t get (const uint32_t i) const
    {
        // Bounds check.
        if (i > mBitSize)
            return 0;
        
        // Generate the required mask for extracting the state.
        uint32_t index = i >> 3;                    // i / 8
        uint8_t  pos   = (uint8_t) (i - (index*8)); // i % 8
        uint8_t  mask  = 1 << pos;
        
        // Extract the state.
        return (A[index] & mask) >> pos;
    }
    
    
    /// @brief  Calculates the hamming weight, or the sum of all elements, in the array.
    /// @return The hamming weight.
    uint32_t hammingWeight (void) const
    {
        // Masks for summing adjacent bits (used subsequently for each step).
        const uint8_t mask_1  = 0x55; // 01010101
        const uint8_t mask_2  = 0x33; // 00110011
        const uint8_t mask_4  = 0x0f; // 00001111
        uint8_t  byteAccumulator = 0;
        uint32_t fullAccumulator = 0;
        uint32_t i;
        
        // TODO THIS IS A NOT VERY GOOD WAY OF DOING THIS.
        for (i = 0; i < mByteSize; i++)
        {
            //This is a naive implementation, shown for comparison,
            //and to help in understanding the better functions.
            //It uses 24 arithmetic operations (shift, add, and).
            //Refer to http://en.wikipedia.org/wiki/Hamming_weight for better implementations (the best does half as many operations).
            byteAccumulator = A[i];
            byteAccumulator = (byteAccumulator & mask_1) + ((byteAccumulator >> 1) & mask_1); // put count of each 2 bits into those 2 bits 
            byteAccumulator = (byteAccumulator & mask_2) + ((byteAccumulator >> 2) & mask_2); // put count of each 4 bits into those 4 bits 
            byteAccumulator = (byteAccumulator & mask_4) + ((byteAccumulator >> 4) & mask_4); // put count of each 8 bits into those 8 bits
            fullAccumulator += byteAccumulator;
        }

        return fullAccumulator;
    }


private:
    uint8_t* A;             ///< The array holding the binary states.
    uint32_t mBitSize;      ///< The number of bits in the array.
    uint32_t mByteSize;     ///< The number of bytes used to store the array.
};

#endif
