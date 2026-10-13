/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rotmat.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

/* Include Files */
#include "rotmat.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : float q_a
 *                float q_b
 *                float q_c
 *                float q_d
 *                float r[9]
 * Return Type  : void
 */
void QuaternionSimCodegenBase_rotmat(float q_a, float q_b, float q_c, float q_d,
                                     float r[9])
{
  float N;
  float ab2;
  float ac2;
  float ad2;
  float bc2;
  float bd2;
  float cd2;
  float qa;
  float qb;
  float qc;
  float qd;
  N = sqrtf(((q_a * q_a + q_b * q_b) + q_c * q_c) + q_d * q_d);
  qa = q_a / N;
  qb = q_b / N;
  qc = q_c / N;
  qd = q_d / N;
  ab2 = qa * qb * 2.0F;
  ac2 = qa * qc * 2.0F;
  ad2 = qa * qd * 2.0F;
  bc2 = qb * qc * 2.0F;
  bd2 = qb * qd * 2.0F;
  cd2 = qc * qd * 2.0F;
  N = qa * qa * 2.0F - 1.0F;
  r[0] = N + qb * qb * 2.0F;
  r[3] = bc2 + ad2;
  r[6] = bd2 - ac2;
  r[1] = bc2 - ad2;
  r[4] = N + qc * qc * 2.0F;
  r[7] = cd2 + ab2;
  r[2] = bd2 + ac2;
  r[5] = cd2 - ab2;
  r[8] = N + qd * qd * 2.0F;
}

/*
 * File trailer for rotmat.c
 *
 * [EOF]
 */
