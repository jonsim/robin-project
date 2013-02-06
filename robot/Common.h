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
#define CHECK_RETURN_XN(rc, what)                                              \
    if (rc != XN_STATUS_OK)                                             \
    {                                                                   \
        printf("%s failed: %s\n", what, xnGetStatusString(rc));         \
        return rc;                                                      \
    }


/*-------------- FUNCTION DEFINITIONS --------------*/
sint8_t  make_sint8_t  (const uint8_t b);
uint16_t make_uint16_t (const uint8_t bh, const uint8_t bl);
sint16_t make_sint16_t (const uint8_t bh, const uint8_t bl);
void     msleep        (const uint32_t msec);

#endif
