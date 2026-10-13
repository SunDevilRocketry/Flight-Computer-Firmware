/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: FC_AHRS_types.h
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

#ifndef FC_AHRS_TYPES_H
#define FC_AHRS_TYPES_H

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_quaternion
#define typedef_quaternion
typedef struct
{
  float a;
  float b;
  float c;
  float d;
} quaternion;
#endif /* typedef_quaternion */

#ifndef typedef_cell_wrap_3
#define typedef_cell_wrap_3
typedef struct
{
  unsigned int f1[8];
} cell_wrap_3;
#endif /* typedef_cell_wrap_3 */

#ifndef typedef_ahrsfilter
#define typedef_ahrsfilter
typedef struct
{
  bool tunablePropertyChanged[12];
  int isInitialized;
  bool isSetupComplete;
  bool TunablePropsChanged;
  cell_wrap_3 inputVarSize[3];
  float SampleRate;
  double AccelerometerNoise;
  double GyroscopeNoise;
  double GyroscopeDriftNoise;
  double LinearAccelerationNoise;
  double LinearAccelerationDecayFactor;
  float pQw[144];
  float pQv[36];
  quaternion pOrientPost;
  quaternion pOrientPrior;
  bool pFirstTime;
  float pSensorPeriod;
  float pKalmanPeriod;
  float pGyroOffset[3];
  float pLinAccelPrior[3];
  float pLinAccelPost[3];
  float pInputPrototype[3];
  double MagnetometerNoise;
  double MagneticDisturbanceNoise;
  double MagneticDisturbanceDecayFactor;
  float ExpectedMagneticFieldStrength;
  double InitialProcessNoise[144];
  float pMagVec[3];
  float pInclinationLimit;
} ahrsfilter;
#endif /* typedef_ahrsfilter */

#ifndef typedef_FC_AHRS
#define typedef_FC_AHRS
typedef struct
{
  ahrsfilter *filter;
  ahrsfilter coder_buffer_pobj0;
} FC_AHRS;
#endif /* typedef_FC_AHRS */

#endif
/*
 * File trailer for FC_AHRS_types.h
 *
 * [EOF]
 */
