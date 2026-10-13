/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FC_AHRS.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

/* Include Files */
#include "FC_AHRS.h"
#include "FC_AHRS_types.h"
#include "ahrsfilter.h"

/* Function Definitions */
/*
 * Arguments    : FC_AHRS *obj
 *                float fs
 * Return Type  : void
 */
void FC_AHRS_FC_AHRS(FC_AHRS *obj, float fs)
{
  ahrsfilter_ahrsfilter(&obj->coder_buffer_pobj0, fs);
  obj->filter = &obj->coder_buffer_pobj0;
  /*  % this should be configurable */
}

/*
 * Arguments    : FC_AHRS *obj
 *                const float accel[3]
 *                const float gyro[3]
 *                const float mag[3]
 *                float orientation[4]
 *                float angular_velo[3]
 * Return Type  : void
 */
void FC_AHRS_ahrs_step(FC_AHRS *obj, const float accel[3], const float gyro[3],
                       const float mag[3], float orientation[4],
                       float angular_velo[3])
{
  ahrsfilter *b_obj;
  float varargout_1_a;
  float varargout_1_b;
  float varargout_1_c;
  float varargout_1_d;
  b_obj = obj->filter;
  varargout_1_a = SystemCore_step(b_obj, accel, gyro, mag, &varargout_1_b,
                                  &varargout_1_c, &varargout_1_d, angular_velo);
  orientation[0] = varargout_1_a;
  orientation[1] = varargout_1_b;
  orientation[2] = varargout_1_c;
  orientation[3] = varargout_1_d;
}

/*
 * File trailer for FC_AHRS.c
 *
 * [EOF]
 */
