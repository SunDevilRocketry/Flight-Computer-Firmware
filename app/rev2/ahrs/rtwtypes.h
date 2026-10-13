/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rtwtypes.h
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

#ifndef RTWTYPES_H
#define RTWTYPES_H

/* Include Files */

#ifndef __cplusplus
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /*=======================================================================*
   * Target hardware information
   *   Device type: ARM Compatible->ARM Cortex-M7 (MPS2)
   *   Number of bits:     char:   8    short:   16    int:  32
   *                       long:  32
   *                       native word size:  32
   *   Byte ordering: LittleEndian
   *   Signed integer division rounds to: Zero
   *   Shift right on a signed integer as arithmetic shift: on
   *=======================================================================*/

  /*=======================================================================*
   * Fixed width word size data types:                                     *
   *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
   *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
   *   real32_T, real64_T           - 32 and 64 bit floating point numbers *
   *=======================================================================*/
  typedef signed char int8_T;
  typedef unsigned char uint8_T;
  typedef short int16_T;
  typedef unsigned short uint16_T;
  typedef int int32_T;
  typedef unsigned int uint32_T;
  typedef float real32_T;
  typedef double real64_T;

  /*===========================================================================*
   * Generic type definitions: real_T, time_T, boolean_T, int_T, uint_T,       *
   *                           ulong_T, char_T and byte_T.                     *
   *===========================================================================*/

  typedef double real_T;
  typedef double time_T;
  typedef bool boolean_T;
  typedef int int_T;
  typedef unsigned int uint_T;
  typedef unsigned long ulong_T;
  typedef char char_T;
  typedef char_T byte_T;

  /*===========================================================================*
   * Complex number type definitions                                           *
   *===========================================================================*/
#ifndef CREAL32_T
  typedef struct
  {
    real32_T re;
    real32_T im;
  } creal32_T;
#define CREAL32_T creal32_T
#endif

#ifndef CREAL64_T
  typedef struct
  {
    real64_T re;
    real64_T im;
  } creal64_T;
#define CREAL64_T creal64_T
#endif

#ifndef CREAL_T
  typedef struct
  {
    real_T re;
    real_T im;
  } creal_T;
#define CREAL_T creal_T
#endif

#ifndef CINT8_T
  typedef struct
  {
    int8_T re;
    int8_T im;
  } cint8_T;
#define CINT8_T cint8_T
#endif

#ifndef CUINT8_T
  typedef struct
  {
    uint8_T re;
    uint8_T im;
  } cuint8_T;
#define CUINT8_T cuint8_T
#endif

#ifndef CINT16_T
  typedef struct
  {
    int16_T re;
    int16_T im;
  } cint16_T;
#define CINT16_T cint16_T
#endif

#ifndef CUINT16_T
  typedef struct
  {
    uint16_T re;
    uint16_T im;
  } cuint16_T;
#define CUINT16_T cuint16_T
#endif

#ifndef CINT32_T
  typedef struct
  {
    int32_T re;
    int32_T im;
  } cint32_T;
#define CINT32_T cint32_T
#endif

#ifndef CUINT32_T
  typedef struct
  {
    uint32_T re;
    uint32_T im;
  } cuint32_T;
#define CUINT32_T cuint32_T
#endif

  /*=======================================================================*
   * Min and Max:                                                          *
   *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
   *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
   *=======================================================================*/

#define MAX_int8_T ((int8_T)(127))
#define MIN_int8_T ((int8_T)(-128))
#define MAX_uint8_T ((uint8_T)(255))
#define MIN_uint8_T ((uint8_T)(0))
#define MAX_int16_T ((int16_T)(32767))
#define MIN_int16_T ((int16_T)(-32768))
#define MAX_uint16_T ((uint16_T)(65535))
#define MIN_uint16_T ((uint16_T)(0))
#define MAX_int32_T ((int32_T)(2147483647))
#define MIN_int32_T ((int32_T)(-2147483647 - 1))
#define MAX_uint32_T ((uint32_T)(0xFFFFFFFFU))
#define MIN_uint32_T ((uint32_T)(0))

/* Logical type definitions */
#if (!defined(__cplusplus)) && (!defined(__true_false_are_keywords)) &&        \
    (!defined(__bool_true_false_are_defined))
#ifndef false
#define false 0U
#endif
#ifndef true
#define true 1U
#endif
#endif

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rtwtypes.h
 *
 * [EOF]
 */
