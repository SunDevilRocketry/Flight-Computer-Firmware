/**
  ******************************************************************************
  * @file           : ahrs.h
  * @brief          : Contains interface for the Attitude and Heading Reference
  *                   System. 
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef AHRS_H
#define AHRS_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes
------------------------------------------------------------------------------*/
#include "FC_AHRS.h"
#include "FC_AHRS_types.h"
#include "math_sdr.h"

/*------------------------------------------------------------------------------
 Public Functions
------------------------------------------------------------------------------*/
void ahrs_init
    (
    FC_AHRS* ahrs,
    float sample_rate
    )
{
FC_AHRS_FC_AHRS(ahrs, sample_rate);
}

void ahrs_step
    (
    FC_AHRS* ahrs,
    VECTOR_3F const* accel,
    VECTOR_3F const* gyro,
    VECTOR_3F const* mag,
    QUAT* orientation,
    VECTOR_3F* angular_velo
    )
{
FC_AHRS_ahrs_step
    (
    ahrs,
    (float*)accel,
    (float*)gyro,
    (float*)mag,
    (float*)orientation,
    (float*)angular_velo
    );
}


#ifdef __cplusplus
}
#endif

#endif /* AHRS_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
