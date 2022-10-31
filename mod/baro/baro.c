/*******************************************************************************
*
* FILE: 
* 		baro.c
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002_rev1.h"
#include "baro.h"


/*------------------------------------------------------------------------------
Global Variables  
------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1; /* MCU I2C handle */


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_device_id                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Gets the device ID of the barometric pressure sensor, primarily used   *
*       to verify that the sensor can be accessed by the MCU                   *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id_ptr /* reference to memory where id is returned */ 
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
HAL_StatusTypeDef hal_status;


/*------------------------------------------------------------------------------
 API Function implementation 
------------------------------------------------------------------------------*/

/* Read baro register with I2C */
hal_status = HAL_I2C_Mem_Read (
                               &hi2c1              ,
                               BARO_I2C_ADDR       ,
                               BARO_REG_CHIP_ID    ,
                               I2C_MEMADD_SIZE_8BIT,
							   baro_id_ptr         ,
							   sizeof( uint8_t )   ,
                               HAL_DEFAULT_TIMEOUT
                              );

/* Check HAL Status and return data if okay */
switch ( hal_status )
	{
	case HAL_OK: 
		return BARO_OK;
		break;

	case HAL_TIMEOUT:
		return BARO_TIMEOUT;
		break;

	case HAL_ERROR:
		return BARO_I2C_ERROR;
		break;

	default:
		return BARO_UNRECOGNIZED_HAL_STATUS;
		break;
	}

} /* baro_get_device_id */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_pressure                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a pressure reading from the sensor                           *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_pressure
	(
    void
	)
{
return BARO_OK;
} /* baro_get_pressure */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_temp                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		retrieves a temperature reading from the sensor                        *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_temp
	(
    void
	)
{
return BARO_OK;
} /* baro_get_temp */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		baro_get_altitude                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		gets the altitude of the rocket from the sensor readouts               *
*                                                                              *
*******************************************************************************/
BARO_STATUS baro_get_altitude
	(
    void
	)
{
return BARO_OK;
} /* baro_get_altitude */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
