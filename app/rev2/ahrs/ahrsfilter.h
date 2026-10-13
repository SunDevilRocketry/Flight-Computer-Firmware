/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ahrsfilter.h
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

#ifndef AHRSFILTER_H
#define AHRSFILTER_H

/* Include Files */
#include "FC_AHRS_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /* Function Declarations */
  float SystemCore_step(ahrsfilter *obj, const float varargin_1[3],
                        const float varargin_2[3], const float varargin_3[3],
                        float *varargout_1_b, float *varargout_1_c,
                        float *varargout_1_d, float varargout_2[3]);

  void ahrsfilter_ahrsfilter(ahrsfilter *obj, float varargin_4);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for ahrsfilter.h
 *
 * [EOF]
 */
