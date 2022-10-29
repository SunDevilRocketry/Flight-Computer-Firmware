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
void baro_get_device_id
	(
    void
	)
{
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
void baro_get_pressure
	(
    void
	)
{
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
void baro_get_temp
	(
    void
	)
{
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
void baro_get_altitude
	(
    void
	)
{
} /* baro_get_altitude */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
