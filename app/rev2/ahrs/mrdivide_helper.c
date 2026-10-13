/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mrdivide_helper.c
 *
 * MATLAB Coder version            : 26.2
 * C/C++ source code generated on  : 12-Oct-2026 19:14:53
 */

/* Include Files */
#include "mrdivide_helper.h"
#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : float A[72]
 *                const float B[36]
 * Return Type  : void
 */
void mrdiv(float A[72], const float B[36])
{
  static float b_A[36];
  float smax;
  int b_k;
  int j;
  int jA;
  int jAcol;
  int k;
  int kBcol;
  int mmj;
  signed char ipiv[6];
  memcpy(&b_A[0], &B[0], 36U * sizeof(float));
  for (k = 0; k < 6; k++)
  {
    ipiv[k] = (signed char)(k + 1);
  }
  for (j = 0; j < 5; j++)
  {
    int b;
    mmj = 4 - j;
    b = j * 7;
    jA = 7 - j;
    jAcol = 0;
    smax = fabsf(b_A[b]);
    for (b_k = 2; b_k < jA; b_k++)
    {
      float s;
      s = fabsf(b_A[(b + b_k) - 1]);
      if (s > smax)
      {
        jAcol = b_k - 1;
        smax = s;
      }
    }
    if (b_A[b + jAcol] != 0.0F)
    {
      if (jAcol != 0)
      {
        jA = j + jAcol;
        ipiv[j] = (signed char)(jA + 1);
        for (k = 0; k < 6; k++)
        {
          jAcol = j + k * 6;
          smax = b_A[jAcol];
          kBcol = jA + k * 6;
          b_A[jAcol] = b_A[kBcol];
          b_A[kBcol] = smax;
        }
      }
      jA = (b - j) + 6;
      for (k = b + 2; k <= jA; k++)
      {
        b_A[k - 1] /= b_A[b];
      }
    }
    jA = b;
    for (b_k = 0; b_k <= mmj; b_k++)
    {
      smax = b_A[(b + b_k * 6) + 6];
      if (smax != 0.0F)
      {
        jAcol = (jA - j) + 12;
        for (k = jA + 8; k <= jAcol; k++)
        {
          b_A[k - 1] += b_A[((b + k) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (j = 0; j < 6; j++)
  {
    mmj = 12 * j - 1;
    jAcol = 6 * j;
    for (b_k = 0; b_k < j; b_k++)
    {
      kBcol = 12 * b_k;
      smax = b_A[b_k + jAcol];
      if (smax != 0.0F)
      {
        for (k = 0; k < 12; k++)
        {
          jA = (k + mmj) + 1;
          A[jA] -= smax * A[k + kBcol];
        }
      }
    }
    smax = 1.0F / b_A[j + jAcol];
    for (k = 0; k < 12; k++)
    {
      jA = (k + mmj) + 1;
      A[jA] *= smax;
    }
  }
  for (b_k = 5; b_k >= 0; b_k--)
  {
    jA = 12 * b_k - 1;
    jAcol = 6 * b_k - 1;
    for (k = b_k + 2; k < 7; k++)
    {
      kBcol = 12 * (k - 1);
      smax = b_A[k + jAcol];
      if (smax != 0.0F)
      {
        for (j = 0; j < 12; j++)
        {
          mmj = (j + jA) + 1;
          A[mmj] -= smax * A[j + kBcol];
        }
      }
    }
  }
  for (k = 4; k >= 0; k--)
  {
    signed char i;
    i = ipiv[k];
    if (i != k + 1)
    {
      for (b_k = 0; b_k < 12; b_k++)
      {
        jA = b_k + 12 * k;
        smax = A[jA];
        jAcol = b_k + 12 * (i - 1);
        A[jA] = A[jAcol];
        A[jAcol] = smax;
      }
    }
  }
}

/*
 * File trailer for mrdivide_helper.c
 *
 * [EOF]
 */
