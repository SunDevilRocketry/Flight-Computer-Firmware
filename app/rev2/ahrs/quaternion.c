/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: quaternion.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

/* Include Files */
#include "quaternion.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : const float varargin_1[3]
 *                float *obj_b
 *                float *obj_c
 *                float *obj_d
 * Return Type  : float
 */
float quaternion_quaternion(const float varargin_1[3], float *obj_b,
                            float *obj_c, float *obj_d)
{
  float ct_tmp;
  float obj_a;
  float st;
  float theta;
  obj_a = 1.0F;
  *obj_b = 0.0F;
  *obj_c = 0.0F;
  *obj_d = 0.0F;
  theta =
      sqrtf((varargin_1[0] * varargin_1[0] + varargin_1[1] * varargin_1[1]) +
            varargin_1[2] * varargin_1[2]);
  ct_tmp = theta / 2.0F;
  st = sinf(ct_tmp);
  if (theta != 0.0F)
  {
    obj_a = cosf(ct_tmp);
    *obj_b = varargin_1[0] / theta * st;
    *obj_c = varargin_1[1] / theta * st;
    *obj_d = varargin_1[2] / theta * st;
  }
  return obj_a;
}

/*
 * File trailer for quaternion.c
 *
 * [EOF]
 */
