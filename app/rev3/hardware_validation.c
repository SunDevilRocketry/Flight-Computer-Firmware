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
#include "flash.h"
#include "error_sdr.h"

/* Global Variables ----------------------------------------------------------*/



/* Private function prototypes -----------------------------------------------*/



/* Procedures ----------------------------------------------------------------*/

static void flash_validation_routine
    (
    void
    )
{
/* Flash */
// Begin Performance Timer
volatile uint32_t start_time = HAL_GetTick();

// Init
HFLASH_BUFFER flash_handle;
memset(&flash_handle, 0, sizeof(HFLASH_BUFFER));
FLASH_STATUS flash_status = flash_init(&flash_handle);
uint8_t flash_buf[FLASH_PAGE_SIZE];
memset(flash_buf, 0, FLASH_PAGE_SIZE);
flash_handle.pbuffer = flash_buf;

assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_INIT_ERROR );

// Set up HW validation routine
flash_status = flash_erase(&flash_handle);

assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );

// Run write sequence (once per page)
flash_buf[0] = 127;
for(uint32_t i = 0; i < FLASH_MAX_ADDR; i+=FLASH_PAGE_SIZE) {
  flash_handle.address = i;
  flash_status = flash_write(&flash_handle);
  assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
}

// Run read sequence and verify (once per page)
memset(flash_buf, 0, FLASH_PAGE_SIZE);
for(uint32_t i = 0; i < FLASH_MAX_ADDR; i+=FLASH_PAGE_SIZE) {
  flash_handle.address = i;
  flash_status = flash_read(&flash_handle, 2);
  assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
  assert_fail_fast( flash_buf[0] == 127, ERROR_FLASH_CMD_ERROR );
  assert_fail_fast( flash_buf[1] == 0xFF, ERROR_FLASH_CMD_ERROR );
}

// Test Block Erasures
// 4K
flash_status = flash_block_erase(0, FLASH_BLOCK_4K);
assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
for(uint32_t i = 0; i < 0x1000 - 1; i+=FLASH_PAGE_SIZE) {
  flash_handle.address = i;
  flash_status = flash_read(&flash_handle, 2);
  assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
  assert_fail_fast( flash_buf[0] == 0xFF, ERROR_FLASH_CMD_ERROR );
}

// 32K
flash_status = flash_block_erase(0, FLASH_BLOCK_32K);
assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
for(uint32_t i = 0; i < 0x8000 - 1; i+=FLASH_PAGE_SIZE) {
  flash_handle.address = i;
  flash_status = flash_read(&flash_handle, 2);
  assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
  assert_fail_fast( flash_buf[0] == 0xFF, ERROR_FLASH_CMD_ERROR );
}

// 64K
flash_status = flash_block_erase(0, FLASH_BLOCK_64K);
assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
for(uint32_t i = 0; i < 0x10000 - 1; i+=FLASH_PAGE_SIZE) {
  flash_handle.address = i;
  flash_status = flash_read(&flash_handle, 2);
  assert_fail_fast( flash_status == FLASH_OK, ERROR_FLASH_CMD_ERROR );
  assert_fail_fast( flash_buf[0] == 0xFF, ERROR_FLASH_CMD_ERROR );
}

// End Performance Timer
volatile uint32_t tdelta = HAL_GetTick() - start_time;
(void)tdelta; // suppress unused

} /* flash_validation_routine */


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

/* flash */
flash_validation_routine();


} /* run_hardware_validation */