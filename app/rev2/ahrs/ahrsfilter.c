/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ahrsfilter.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

/* Include Files */
#include "ahrsfilter.h"
#include "FC_AHRS_types.h"
#include "mrdivide_helper.h"
#include "quaternion.h"
#include "rotmat.h"
#include <math.h>
#include <string.h>

/* Variable Definitions */
static const signed char iv[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

/* Function Declarations */
static float AHRSFilterBase_stepImpl(ahrsfilter *obj, const float accelIn[3],
                                     const float gyroIn[3],
                                     const float magIn[3], float *orientOut_b,
                                     float *orientOut_c, float *orientOut_d,
                                     float av[3]);

/* Function Definitions */
/*
 * Arguments    : ahrsfilter *obj
 *                const float accelIn[3]
 *                const float gyroIn[3]
 *                const float magIn[3]
 *                float *orientOut_b
 *                float *orientOut_c
 *                float *orientOut_d
 *                float av[3]
 * Return Type  : float
 */
static float AHRSFilterBase_stepImpl(ahrsfilter *obj, const float accelIn[3],
                                     const float gyroIn[3],
                                     const float magIn[3], float *orientOut_b,
                                     float *orientOut_c, float *orientOut_d,
                                     float av[3])
{
  static float Ppost[144];
  static float H[72];
  static float b_obj[72];
  static float resCovar_tmp[72];
  static float y_tmp[72];
  static float b_resCovar_tmp[36];
  static float fv[36];
  static const signed char b_iv[9] = {-1, 0, 0, 0, -1, 0, 0, 0, -1};
  double a;
  float xe_post[12];
  float Rprior[9];
  float b_h2[9];
  float h2[9];
  float res[6];
  float Reast[3];
  float gravityAccelGyroDiff[3];
  float deltaq_a;
  float deltaq_c;
  float deltaq_d;
  float f;
  float f1;
  float f2;
  float linAccelErr_idx_0;
  float linAccelErr_idx_1;
  float linAccelErr_idx_2;
  float orientOut_a;
  float t;
  float t0_a;
  float t0_b;
  float t0_c;
  float xb;
  float xc;
  int i;
  int i1;
  int xi;
  int xpageoffset;
  bool isJamming;
  av[0] = gyroIn[0] - obj->pGyroOffset[0];
  av[1] = gyroIn[1] - obj->pGyroOffset[1];
  av[2] = gyroIn[2] - obj->pGyroOffset[2];
  if (obj->pFirstTime)
  {
    Reast[0] = accelIn[1] * magIn[2] - magIn[1] * accelIn[2];
    Reast[1] = magIn[0] * accelIn[2] - accelIn[0] * magIn[2];
    Reast[2] = accelIn[0] * magIn[1] - magIn[0] * accelIn[1];
    h2[6] = accelIn[0];
    h2[3] = Reast[0];
    h2[7] = accelIn[1];
    h2[4] = Reast[1];
    h2[8] = accelIn[2];
    h2[5] = Reast[2];
    h2[0] = Reast[1] * accelIn[2] - accelIn[1] * Reast[2];
    h2[1] = accelIn[0] * Reast[2] - Reast[0] * accelIn[2];
    h2[2] = Reast[0] * accelIn[1] - accelIn[0] * Reast[1];
    for (xi = 0; xi < 9; xi++)
    {
      orientOut_a = h2[xi];
      Rprior[xi] = orientOut_a * orientOut_a;
    }
    for (xi = 0; xi < 3; xi++)
    {
      xpageoffset = xi * 3;
      Reast[xi] = sqrtf((Rprior[xpageoffset] + Rprior[xpageoffset + 1]) +
                        Rprior[xpageoffset + 2]);
    }
    for (xi = 0; xi < 9; xi++)
    {
      Rprior[xi] = h2[xi];
    }
    for (xi = 0; xi < 3; xi++)
    {
      orientOut_a = Reast[xi];
      h2[3 * xi] = Rprior[3 * xi] / orientOut_a;
      xpageoffset = 3 * xi + 1;
      h2[xpageoffset] = Rprior[xpageoffset] / orientOut_a;
      xpageoffset = 3 * xi + 2;
      h2[xpageoffset] = Rprior[xpageoffset] / orientOut_a;
    }
    orientOut_a = (h2[0] + h2[4]) + h2[8];
    xb = (2.0F * orientOut_a + 1.0F) - orientOut_a;
    xpageoffset = 1;
    xc = (2.0F * h2[0] + 1.0F) - orientOut_a;
    if (xb < xc)
    {
      xb = xc;
      xpageoffset = 2;
    }
    xc = (2.0F * h2[4] + 1.0F) - orientOut_a;
    if (xb < xc)
    {
      xb = xc;
      xpageoffset = 3;
    }
    xc = (2.0F * h2[8] + 1.0F) - orientOut_a;
    if (xb < xc)
    {
      xb = xc;
      xpageoffset = 4;
    }
    switch (xpageoffset)
    {
    case 1:
      orientOut_a = sqrtf(xb);
      deltaq_c = 0.5F * orientOut_a;
      xc = 0.5F / orientOut_a;
      deltaq_d = xc * (h2[7] - h2[5]);
      t = xc * (h2[2] - h2[6]);
      xb = xc * (h2[3] - h2[1]);
      break;
    case 2:
      orientOut_a = sqrtf(xb);
      deltaq_d = 0.5F * orientOut_a;
      xb = 0.5F / orientOut_a;
      deltaq_c = xb * (h2[7] - h2[5]);
      t = xb * (h2[1] + h2[3]);
      xb *= h2[2] + h2[6];
      break;
    case 3:
      orientOut_a = sqrtf(xb);
      t = 0.5F * orientOut_a;
      orientOut_a = 0.5F / orientOut_a;
      deltaq_c = orientOut_a * (h2[2] - h2[6]);
      deltaq_d = orientOut_a * (h2[1] + h2[3]);
      xb = orientOut_a * (h2[5] + h2[7]);
      break;
    default:
      orientOut_a = sqrtf(xb);
      xb = 0.5F * orientOut_a;
      orientOut_a = 0.5F / orientOut_a;
      deltaq_c = orientOut_a * (h2[3] - h2[1]);
      deltaq_d = orientOut_a * (h2[2] + h2[6]);
      t = orientOut_a * (h2[5] + h2[7]);
      break;
    }
    if (deltaq_c < 0.0F)
    {
      deltaq_c = -deltaq_c;
      deltaq_d = -deltaq_d;
      t = -t;
      xb = -xb;
    }
    obj->pOrientPost.a = deltaq_c;
    obj->pOrientPost.b = deltaq_d;
    obj->pOrientPost.c = t;
    obj->pOrientPost.d = xb;
  }
  gravityAccelGyroDiff[0] =
      (gyroIn[0] - obj->pGyroOffset[0]) * obj->pSensorPeriod;
  gravityAccelGyroDiff[1] =
      (gyroIn[1] - obj->pGyroOffset[1]) * obj->pSensorPeriod;
  gravityAccelGyroDiff[2] =
      (gyroIn[2] - obj->pGyroOffset[2]) * obj->pSensorPeriod;
  deltaq_a =
      quaternion_quaternion(gravityAccelGyroDiff, &t, &deltaq_c, &deltaq_d);
  orientOut_a = obj->pOrientPost.a;
  xb = obj->pOrientPost.b;
  xc = obj->pOrientPost.c;
  obj->pOrientPost.a =
      ((obj->pOrientPost.a * deltaq_a - obj->pOrientPost.b * t) -
       obj->pOrientPost.c * deltaq_c) -
      obj->pOrientPost.d * deltaq_d;
  obj->pOrientPost.b = ((orientOut_a * t + obj->pOrientPost.b * deltaq_a) +
                        obj->pOrientPost.c * deltaq_d) -
                       obj->pOrientPost.d * deltaq_c;
  obj->pOrientPost.c = ((orientOut_a * deltaq_c - xb * deltaq_d) +
                        obj->pOrientPost.c * deltaq_a) +
                       obj->pOrientPost.d * t;
  obj->pOrientPost.d = ((orientOut_a * deltaq_d + xb * deltaq_c) - xc * t) +
                       obj->pOrientPost.d * deltaq_a;
  if (obj->pOrientPost.a < 0.0F)
  {
    obj->pOrientPost.a = -obj->pOrientPost.a;
    obj->pOrientPost.b = -obj->pOrientPost.b;
    obj->pOrientPost.c = -obj->pOrientPost.c;
    obj->pOrientPost.d = -obj->pOrientPost.d;
  }
  QuaternionSimCodegenBase_rotmat(obj->pOrientPost.a, obj->pOrientPost.b,
                                  obj->pOrientPost.c, obj->pOrientPost.d,
                                  Rprior);
  a = obj->LinearAccelerationDecayFactor;
  memset(&Reast[0], 0, 3U * sizeof(float));
  xc = Reast[0];
  t = Reast[1];
  deltaq_c = Reast[2];
  for (xi = 0; xi < 3; xi++)
  {
    orientOut_a = Rprior[xi + 6] * 9.81F;
    Reast[xi] = orientOut_a;
    xb = (float)a * obj->pLinAccelPost[xi];
    obj->pLinAccelPrior[xi] = xb;
    gravityAccelGyroDiff[xi] = (accelIn[xi] + xb) - orientOut_a;
    orientOut_a = obj->pMagVec[xi];
    xc += Rprior[3 * xi] * orientOut_a;
    t += Rprior[3 * xi + 1] * orientOut_a;
    deltaq_c += Rprior[3 * xi + 2] * orientOut_a;
  }
  for (xi = 0; xi < 9; xi++)
  {
    Rprior[xi] = 0.0F;
  }
  Rprior[3] = Reast[2];
  Rprior[6] = -Reast[1];
  Rprior[7] = Reast[0];
  for (xi = 0; xi < 3; xi++)
  {
    h2[3 * xi] = Rprior[3 * xi];
    xpageoffset = 3 * xi + 1;
    h2[xpageoffset] = Rprior[xpageoffset] - Rprior[xi + 3];
    xpageoffset = 3 * xi + 2;
    h2[xpageoffset] = Rprior[xpageoffset] - Rprior[xi + 6];
  }
  for (xi = 0; xi < 9; xi++)
  {
    Rprior[xi] = h2[xi];
    h2[xi] = 0.0F;
  }
  h2[3] = deltaq_c;
  h2[6] = -t;
  h2[7] = xc;
  for (xi = 0; xi < 3; xi++)
  {
    b_h2[3 * xi] = h2[3 * xi] - h2[xi];
    xpageoffset = 3 * xi + 1;
    b_h2[xpageoffset] = h2[xpageoffset] - h2[xi + 3];
    xpageoffset = 3 * xi + 2;
    b_h2[xpageoffset] = h2[xpageoffset] - h2[xi + 6];
  }
  for (xi = 0; xi < 9; xi++)
  {
    h2[xi] = b_h2[xi];
  }
  for (xi = 0; xi < 3; xi++)
  {
    int H_tmp;
    int b_H_tmp;
    int c_H_tmp;
    orientOut_a = Rprior[3 * xi];
    H[6 * xi] = orientOut_a;
    H_tmp = 6 * (xi + 3);
    H[H_tmp] = -orientOut_a * obj->pKalmanPeriod;
    b_H_tmp = 6 * (xi + 6);
    H[b_H_tmp] = iv[3 * xi];
    c_H_tmp = 6 * (xi + 9);
    H[c_H_tmp] = 0.0F;
    orientOut_a = h2[3 * xi];
    H[6 * xi + 3] = orientOut_a;
    H[H_tmp + 3] = -orientOut_a * obj->pKalmanPeriod;
    H[b_H_tmp + 3] = 0.0F;
    H[c_H_tmp + 3] = b_iv[3 * xi];
    xpageoffset = 3 * xi + 1;
    orientOut_a = Rprior[xpageoffset];
    H[6 * xi + 1] = orientOut_a;
    H[H_tmp + 1] = -orientOut_a * obj->pKalmanPeriod;
    H[b_H_tmp + 1] = iv[xpageoffset];
    H[c_H_tmp + 1] = 0.0F;
    orientOut_a = h2[xpageoffset];
    H[6 * xi + 4] = orientOut_a;
    H[H_tmp + 4] = -orientOut_a * obj->pKalmanPeriod;
    H[b_H_tmp + 4] = 0.0F;
    H[c_H_tmp + 4] = b_iv[xpageoffset];
    xpageoffset = 3 * xi + 2;
    orientOut_a = Rprior[xpageoffset];
    H[6 * xi + 2] = orientOut_a;
    H[H_tmp + 2] = -orientOut_a * obj->pKalmanPeriod;
    H[b_H_tmp + 2] = iv[xpageoffset];
    H[c_H_tmp + 2] = 0.0F;
    orientOut_a = h2[xpageoffset];
    H[6 * xi + 5] = orientOut_a;
    H[H_tmp + 5] = -orientOut_a * obj->pKalmanPeriod;
    H[b_H_tmp + 5] = 0.0F;
    H[c_H_tmp + 5] = b_iv[xpageoffset];
  }
  memset(&resCovar_tmp[0], 0, 72U * sizeof(float));
  for (xi = 0; xi < 12; xi++)
  {
    for (i = 0; i < 12; i++)
    {
      orientOut_a = obj->pQw[i + 12 * xi];
      for (i1 = 0; i1 < 6; i1++)
      {
        xpageoffset = i1 + 6 * xi;
        resCovar_tmp[xpageoffset] += H[i1 + 6 * i] * orientOut_a;
      }
    }
  }
  memset(&b_obj[0], 0, 72U * sizeof(float));
  for (xi = 0; xi < 6; xi++)
  {
    for (i = 0; i < 12; i++)
    {
      orientOut_a = H[xi + 6 * i];
      y_tmp[i + 12 * xi] = orientOut_a;
      for (i1 = 0; i1 < 12; i1++)
      {
        xpageoffset = i1 + 12 * xi;
        b_obj[xpageoffset] += obj->pQw[i1 + 12 * i] * orientOut_a;
      }
    }
  }
  memcpy(&H[0], &b_obj[0], 72U * sizeof(float));
  memset(&fv[0], 0, 36U * sizeof(float));
  for (xi = 0; xi < 6; xi++)
  {
    for (i = 0; i < 12; i++)
    {
      orientOut_a = y_tmp[i + 12 * xi];
      for (i1 = 0; i1 < 6; i1++)
      {
        xpageoffset = i1 + 6 * xi;
        fv[xpageoffset] += resCovar_tmp[i1 + 6 * i] * orientOut_a;
      }
    }
  }
  for (xi = 0; xi < 6; xi++)
  {
    for (i = 0; i < 6; i++)
    {
      xpageoffset = xi + 6 * i;
      b_resCovar_tmp[i + 6 * xi] = fv[xpageoffset] + obj->pQv[xpageoffset];
    }
  }
  mrdiv(H, b_resCovar_tmp);
  res[0] = gravityAccelGyroDiff[0];
  res[3] = magIn[0] - xc;
  res[1] = gravityAccelGyroDiff[1];
  res[4] = magIn[1] - t;
  res[2] = gravityAccelGyroDiff[2];
  res[5] = magIn[2] - deltaq_c;
  memset(&Reast[0], 0, 3U * sizeof(float));
  f = Reast[0];
  f1 = Reast[1];
  f2 = Reast[2];
  for (xi = 0; xi < 6; xi++)
  {
    orientOut_a = res[xi];
    f += H[12 * xi + 9] * orientOut_a;
    f1 += H[12 * xi + 10] * orientOut_a;
    f2 += H[12 * xi + 11] * orientOut_a;
  }
  orientOut_a = 1.2924697E-26F;
  xb = fabsf(f);
  if (xb > 1.2924697E-26F)
  {
    xc = 1.0F;
    orientOut_a = xb;
  }
  else
  {
    t = xb / 1.2924697E-26F;
    xc = t * t;
  }
  xb = fabsf(f1);
  if (xb > orientOut_a)
  {
    t = orientOut_a / xb;
    xc = xc * t * t + 1.0F;
    orientOut_a = xb;
  }
  else
  {
    t = xb / orientOut_a;
    xc += t * t;
  }
  xb = fabsf(f2);
  if (xb > orientOut_a)
  {
    t = orientOut_a / xb;
    xc = xc * t * t + 1.0F;
    orientOut_a = xb;
  }
  else
  {
    t = xb / orientOut_a;
    xc += t * t;
  }
  orientOut_a *= sqrtf(xc);
  xb = obj->ExpectedMagneticFieldStrength;
  isJamming = (orientOut_a * orientOut_a > 4.0F * (xb * xb));
  if (isJamming)
  {
    memset(&h2[0], 0, 9U * sizeof(float));
    for (xi = 0; xi < 3; xi++)
    {
      orientOut_a = gravityAccelGyroDiff[xi];
      for (i = 0; i < 9; i++)
      {
        h2[i] += H[i + 12 * xi] * orientOut_a;
      }
    }
    for (xi = 0; xi < 9; xi++)
    {
      Rprior[xi] = h2[xi];
    }
    Reast[0] = Rprior[0];
    gravityAccelGyroDiff[0] = Rprior[3];
    linAccelErr_idx_0 = Rprior[6];
    Reast[1] = Rprior[1];
    gravityAccelGyroDiff[1] = Rprior[4];
    linAccelErr_idx_1 = Rprior[7];
    Reast[2] = Rprior[2];
    gravityAccelGyroDiff[2] = Rprior[5];
    linAccelErr_idx_2 = Rprior[8];
  }
  else
  {
    memset(&xe_post[0], 0, 12U * sizeof(float));
    for (xi = 0; xi < 6; xi++)
    {
      xb = res[xi];
      for (i = 0; i < 12; i++)
      {
        xe_post[i] += H[i + 12 * xi] * xb;
      }
    }
    Reast[0] = xe_post[0];
    gravityAccelGyroDiff[0] = xe_post[3];
    linAccelErr_idx_0 = xe_post[6];
    Reast[1] = xe_post[1];
    gravityAccelGyroDiff[1] = xe_post[4];
    linAccelErr_idx_1 = xe_post[7];
    Reast[2] = xe_post[2];
    gravityAccelGyroDiff[2] = xe_post[5];
    linAccelErr_idx_2 = xe_post[8];
  }
  deltaq_a = quaternion_quaternion(Reast, &t, &deltaq_c, &deltaq_d);
  t = -t;
  deltaq_c = -deltaq_c;
  deltaq_d = -deltaq_d;
  t0_a = ((obj->pOrientPost.a * deltaq_a - obj->pOrientPost.b * t) -
          obj->pOrientPost.c * deltaq_c) -
         obj->pOrientPost.d * deltaq_d;
  t0_b = ((obj->pOrientPost.a * t + obj->pOrientPost.b * deltaq_a) +
          obj->pOrientPost.c * deltaq_d) -
         obj->pOrientPost.d * deltaq_c;
  t0_c = ((obj->pOrientPost.a * deltaq_c - obj->pOrientPost.b * deltaq_d) +
          obj->pOrientPost.c * deltaq_a) +
         obj->pOrientPost.d * t;
  xc = ((obj->pOrientPost.a * deltaq_d + obj->pOrientPost.b * deltaq_c) -
        obj->pOrientPost.c * t) +
       obj->pOrientPost.d * deltaq_a;
  if (t0_a < 0.0F)
  {
    t0_a = -t0_a;
    t0_b = -t0_b;
    t0_c = -t0_c;
    xc = -xc;
  }
  orientOut_a = sqrtf(((t0_a * t0_a + t0_b * t0_b) + t0_c * t0_c) + xc * xc);
  t0_a /= orientOut_a;
  t0_b /= orientOut_a;
  t0_c /= orientOut_a;
  xc /= orientOut_a;
  if (!isJamming)
  {
    QuaternionSimCodegenBase_rotmat(t0_a, t0_b, t0_c, xc, Rprior);
    for (xi = 0; xi < 3; xi++)
    {
      Reast[xi] =
          obj->pMagVec[xi] - ((Rprior[3 * xi] * f + Rprior[3 * xi + 1] * f1) +
                              Rprior[3 * xi + 2] * f2);
    }
    orientOut_a = atan2f(Reast[2], Reast[0]);
    if (orientOut_a < -obj->pInclinationLimit)
    {
      orientOut_a = -1.5707964F;
    }
    if (orientOut_a > obj->pInclinationLimit)
    {
      orientOut_a = 1.5707964F;
    }
    obj->pMagVec[1] = 0.0F;
    obj->pMagVec[0] = cosf(orientOut_a);
    obj->pMagVec[2] = sinf(orientOut_a);
    obj->pMagVec[0] *= obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[1] *= obj->ExpectedMagneticFieldStrength;
    obj->pMagVec[2] *= obj->ExpectedMagneticFieldStrength;
  }
  for (i = 0; i < 12; i++)
  {
    for (i1 = 0; i1 < 12; i1++)
    {
      orientOut_a = 0.0F;
      for (xi = 0; xi < 6; xi++)
      {
        orientOut_a += H[i + 12 * xi] * resCovar_tmp[xi + 6 * i1];
      }
      xpageoffset = i + 12 * i1;
      Ppost[xpageoffset] = obj->pQw[xpageoffset] - orientOut_a;
    }
  }
  double b_a;
  memset(&obj->pQw[0], 0, 144U * sizeof(float));
  orientOut_a = obj->pKalmanPeriod;
  orientOut_a *= orientOut_a;
  xb = (float)(obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
  obj->pQw[0] = Ppost[0] + orientOut_a * (Ppost[39] + xb);
  obj->pQw[39] = Ppost[39] + (float)obj->GyroscopeDriftNoise;
  obj->pQw[13] = Ppost[13] + orientOut_a * (Ppost[52] + xb);
  obj->pQw[52] = Ppost[52] + (float)obj->GyroscopeDriftNoise;
  obj->pQw[26] = Ppost[26] + orientOut_a * (Ppost[65] + xb);
  obj->pQw[65] = Ppost[65] + (float)obj->GyroscopeDriftNoise;
  orientOut_a = -obj->pKalmanPeriod;
  Reast[0] = orientOut_a * obj->pQw[39];
  Reast[1] = orientOut_a * obj->pQw[52];
  Reast[2] = orientOut_a * obj->pQw[65];
  b_a = a * a;
  a = obj->MagneticDisturbanceDecayFactor;
  a *= a;
  obj->pFirstTime = false;
  obj->pOrientPrior = obj->pOrientPost;
  obj->pOrientPost.a = t0_a;
  obj->pOrientPost.b = t0_b;
  obj->pOrientPost.c = t0_c;
  obj->pOrientPost.d = xc;
  obj->pQw[3] = Reast[0];
  obj->pQw[36] = Reast[0];
  obj->pQw[78] = (float)b_a * Ppost[78] + (float)obj->LinearAccelerationNoise;
  obj->pQw[117] = (float)a * Ppost[117] + (float)obj->MagneticDisturbanceNoise;
  obj->pGyroOffset[0] -= gravityAccelGyroDiff[0];
  obj->pLinAccelPost[0] = obj->pLinAccelPrior[0] - linAccelErr_idx_0;
  obj->pQw[16] = Reast[1];
  obj->pQw[49] = Reast[1];
  obj->pQw[91] = (float)b_a * Ppost[91] + (float)obj->LinearAccelerationNoise;
  obj->pQw[130] = (float)a * Ppost[130] + (float)obj->MagneticDisturbanceNoise;
  obj->pGyroOffset[1] -= gravityAccelGyroDiff[1];
  obj->pLinAccelPost[1] = obj->pLinAccelPrior[1] - linAccelErr_idx_1;
  obj->pQw[29] = Reast[2];
  obj->pQw[62] = Reast[2];
  obj->pQw[104] = (float)b_a * Ppost[104] + (float)obj->LinearAccelerationNoise;
  obj->pQw[143] = (float)a * Ppost[143] + (float)obj->MagneticDisturbanceNoise;
  obj->pGyroOffset[2] -= gravityAccelGyroDiff[2];
  obj->pLinAccelPost[2] = obj->pLinAccelPrior[2] - linAccelErr_idx_2;
  orientOut_a = obj->pOrientPost.a;
  *orientOut_b = obj->pOrientPost.b;
  *orientOut_c = obj->pOrientPost.c;
  *orientOut_d = obj->pOrientPost.d;
  return orientOut_a;
}

/*
 * Arguments    : ahrsfilter *obj
 *                const float varargin_1[3]
 *                const float varargin_2[3]
 *                const float varargin_3[3]
 *                float *varargout_1_b
 *                float *varargout_1_c
 *                float *varargout_1_d
 *                float varargout_2[3]
 * Return Type  : float
 */
float SystemCore_step(ahrsfilter *obj, const float varargin_1[3],
                      const float varargin_2[3], const float varargin_3[3],
                      float *varargout_1_b, float *varargout_1_c,
                      float *varargout_1_d, float varargout_2[3])
{
  float a;
  float varargout_1_a;
  int i;
  int i1;
  int i2;
  signed char b[9];
  if (obj->isInitialized != 1)
  {
    obj->isInitialized = 1;
    obj->pSensorPeriod = 1.0F / obj->SampleRate;
    obj->pKalmanPeriod = obj->pSensorPeriod;
    obj->isSetupComplete = true;
    obj->TunablePropsChanged = false;
    obj->pOrientPost.a = 1.0F;
    obj->pOrientPost.b = 0.0F;
    obj->pOrientPost.c = 0.0F;
    obj->pOrientPost.d = 0.0F;
    obj->pInputPrototype[0] = varargin_1[0];
    obj->pGyroOffset[0] = 0.0F;
    obj->pInputPrototype[1] = varargin_1[1];
    obj->pGyroOffset[1] = 0.0F;
    obj->pMagVec[1] = 0.0F;
    obj->pInputPrototype[2] = varargin_1[2];
    obj->pGyroOffset[2] = 0.0F;
    obj->pMagVec[2] = 0.0F;
    obj->pMagVec[0] = obj->ExpectedMagneticFieldStrength;
    varargout_1_a = obj->pKalmanPeriod;
    a = (float)(obj->AccelerometerNoise + obj->LinearAccelerationNoise) +
        varargout_1_a * varargout_1_a *
            (float)(obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
    varargout_1_a =
        (float)(obj->MagnetometerNoise + obj->MagneticDisturbanceNoise) +
        varargout_1_a * varargout_1_a *
            (float)(obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
    for (i = 0; i < 9; i++)
    {
      b[i] = 0;
    }
    b[0] = 1;
    b[4] = 1;
    b[8] = 1;
    memset(&obj->pQv[0], 0, 36U * sizeof(float));
    for (i = 0; i < 3; i++)
    {
      obj->pQv[6 * i] = a * (float)b[3 * i];
      i1 = 6 * (i + 3);
      obj->pQv[i1 + 3] = varargout_1_a * (float)iv[3 * i];
      i2 = 3 * i + 1;
      obj->pQv[6 * i + 1] = a * (float)b[i2];
      obj->pQv[i1 + 4] = varargout_1_a * (float)iv[i2];
      i2 = 3 * i + 2;
      obj->pQv[6 * i + 2] = a * (float)b[i2];
      obj->pQv[i1 + 5] = varargout_1_a * (float)iv[i2];
    }
    for (i = 0; i < 144; i++)
    {
      obj->pQw[i] = (float)obj->InitialProcessNoise[i];
    }
    obj->pLinAccelPost[0] = 0.0F;
    obj->pLinAccelPost[1] = 0.0F;
    obj->pLinAccelPost[2] = 0.0F;
    obj->pFirstTime = true;
    obj->pInclinationLimit = 1.5707964F;
  }
  if (obj->TunablePropsChanged)
  {
    obj->TunablePropsChanged = false;
    obj->pSensorPeriod = 1.0F / obj->SampleRate;
    obj->pKalmanPeriod = obj->pSensorPeriod;
    varargout_1_a = obj->pKalmanPeriod;
    a = (float)(obj->AccelerometerNoise + obj->LinearAccelerationNoise) +
        varargout_1_a * varargout_1_a *
            (float)(obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
    varargout_1_a =
        (float)(obj->MagnetometerNoise + obj->MagneticDisturbanceNoise) +
        varargout_1_a * varargout_1_a *
            (float)(obj->GyroscopeDriftNoise + obj->GyroscopeNoise);
    for (i = 0; i < 9; i++)
    {
      b[i] = 0;
    }
    b[0] = 1;
    b[4] = 1;
    b[8] = 1;
    memset(&obj->pQv[0], 0, 36U * sizeof(float));
    for (i = 0; i < 3; i++)
    {
      obj->pQv[6 * i] = a * (float)b[3 * i];
      i1 = 6 * (i + 3);
      obj->pQv[i1 + 3] = varargout_1_a * (float)iv[3 * i];
      i2 = 3 * i + 1;
      obj->pQv[6 * i + 1] = a * (float)b[i2];
      obj->pQv[i1 + 4] = varargout_1_a * (float)iv[i2];
      i2 = 3 * i + 2;
      obj->pQv[6 * i + 2] = a * (float)b[i2];
      obj->pQv[i1 + 5] = varargout_1_a * (float)iv[i2];
    }
    for (i = 0; i < 12; i++)
    {
      obj->tunablePropertyChanged[i] = false;
    }
  }
  return AHRSFilterBase_stepImpl(obj, varargin_1, varargin_2, varargin_3,
                                 varargout_1_b, varargout_1_c, varargout_1_d,
                                 varargout_2);
}

/*
 * Arguments    : ahrsfilter *obj
 *                float varargin_4
 * Return Type  : void
 */
void ahrsfilter_ahrsfilter(ahrsfilter *obj, float varargin_4)
{
  static const double dv[144] = {6.092348395734171E-6,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 6.092348395734171E-6,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 6.092348395734171E-6,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 7.615435494667714E-5,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 7.615435494667714E-5,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 7.615435494667714E-5,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.009623610000000001,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.009623610000000001,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.009623610000000001,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.6,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.6,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.0,
                                 0.6};
  int i;
  memcpy(&obj->InitialProcessNoise[0], &dv[0], 144U * sizeof(double));
  obj->MagnetometerNoise = 0.1;
  obj->MagneticDisturbanceNoise = 0.5;
  obj->MagneticDisturbanceDecayFactor = 0.5;
  obj->AccelerometerNoise = 0.0001924722;
  obj->GyroscopeNoise = 9.1385E-5;
  obj->GyroscopeDriftNoise = 3.0462E-13;
  obj->LinearAccelerationNoise = 0.009623610000000001;
  obj->LinearAccelerationDecayFactor = 0.5;
  obj->TunablePropsChanged = false;
  obj->isInitialized = 0;
  for (i = 0; i < 12; i++)
  {
    obj->tunablePropertyChanged[i] = false;
  }
  if (obj->isInitialized == 1)
  {
    obj->TunablePropsChanged = true;
    obj->tunablePropertyChanged[2] = true;
  }
  obj->SampleRate = varargin_4;
  if (obj->isInitialized == 1)
  {
    obj->TunablePropsChanged = true;
    obj->tunablePropertyChanged[6] = true;
  }
  obj->ExpectedMagneticFieldStrength = 45.6F;
  if (obj->isInitialized == 1)
  {
    obj->TunablePropsChanged = true;
    obj->tunablePropertyChanged[1] = true;
  }
}

/*
 * File trailer for ahrsfilter.c
 *
 * [EOF]
 */
