/**
 * @file        Common.h
 * @brief       Set of common functions, definitions and includes.
 */
#ifndef COMMON_H
#define COMMON_H

/*-------------------- INCLUDES --------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>


/*-------------------- DEFINES  --------------------*/
typedef unsigned char       uint8_t;
typedef unsigned short      uint16_t;
typedef unsigned long       uint32_t;
//typedef unsigned long long  uint64_t;
typedef signed char         sint8_t;
typedef signed short        sint16_t;
typedef signed long         sint32_t;
//typedef signed long long    sint64_t;


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


/*-------------- FUNCTION DEFINITIONS --------------*/
sint8_t  make_sint8_t  (const uint8_t b);
uint16_t make_uint16_t (const uint8_t bh, const uint8_t bl);
sint16_t make_sint16_t (const uint8_t bh, const uint8_t bl);

#endif
