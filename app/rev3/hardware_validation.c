/**
  ******************************************************************************
  * @file           : hardware_validation.c
  * @brief          : Routines to validate components on the FC.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Sun Devil Rocketry.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "led.h"

/* Global Variables ----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/



/* Procedures ----------------------------------------------------------------*/

void run_hardware_validation
    (
    void
    ) 
{
/* LED */
led_set_color(LED_WHITE);
HAL_Delay(1000);
led_set_color(LED_RED);
HAL_Delay(1000);
led_set_color(LED_BLUE);
HAL_Delay(1000);
led_set_color(LED_GREEN);
HAL_Delay(1000);


} /* run_hardware_validation */