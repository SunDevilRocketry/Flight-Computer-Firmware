/**
  ******************************************************************************
  * @file           : scheduler_contract.c
  * @brief          : TODO
  * @attention      : Criticality: FQ - Flight Qualified
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

/*------------------------------------------------------------------------------
 Standard Includes
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "scheduler.h"
#include "math_sdr.h"
#include "buzzer.h"
#include "sdr_pin_defines_A0002.h"
#include "led.h"


/*------------------------------------------------------------------------------
 Callback Function Prototypes
------------------------------------------------------------------------------*/
static uint16_t task_callback_start_buzz(void);
static uint16_t task_callback_stop_buzz(void);

/*------------------------------------------------------------------------------
 Callback Table                                                                  
------------------------------------------------------------------------------*/
SCHEDULED_TASK task_scheduler_table[] = 
    {
    { TASK_START_BUZZ,  0, task_callback_start_buzz   },
    { TASK_STOP_BUZZ,   0, task_callback_stop_buzz    }
    };
uint8_t task_scheduler_table_size = array_size(task_scheduler_table);


/*------------------------------------------------------------------------------
 Callback Implementations
------------------------------------------------------------------------------*/
static uint16_t task_callback_start_buzz(void)
{
led_set_color( LED_BLUE );

HAL_StatusTypeDef hal_status; /* Return codes from HAL API */

hal_status = HAL_TIM_PWM_Start( &(BUZZ_TIM), BUZZ_TIM_CHANNEL );
if ( hal_status != HAL_OK )
	{
	return BUZZ_HAL_ERROR;
	}
else
    {
    return BUZZ_OK;
    }

}


static uint16_t task_callback_stop_buzz(void)
{
HAL_StatusTypeDef hal_status; /* Return codes from HAL API */

led_set_color( LED_YELLOW );


hal_status = HAL_TIM_PWM_Stop( &( BUZZ_TIM ), BUZZ_TIM_CHANNEL );
if ( hal_status != HAL_OK )
	{
	return BUZZ_HAL_ERROR;
	}
else
	{
	return BUZZ_OK;
	}

}

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/