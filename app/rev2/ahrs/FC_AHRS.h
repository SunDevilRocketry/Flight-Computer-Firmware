/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FC_AHRS.h
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

#ifndef FC_AHRS_H
#define FC_AHRS_H

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
  extern void FC_AHRS_FC_AHRS(FC_AHRS *obj, float fs);

  extern void FC_AHRS_ahrs_step(FC_AHRS *obj, const float accel[3],
                                const float gyro[3], const float mag[3],
                                float orientation[4], float angular_velo[3]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for FC_AHRS.h
 *
 * [EOF]
 */
