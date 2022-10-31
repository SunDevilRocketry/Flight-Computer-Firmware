/*******************************************************************************
*
* FILE: 
* 		baro.h
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BARO_H 
#define BARO_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return codes for API functions */
typedef enum BARO_STATUS
	{
	BARO_OK                     ,
	BARO_FAIL                   , 
	BARO_TIMEOUT                ,
	BARO_UNRECOGNIZED_HAL_STATUS,
	BARO_I2C_ERROR
	} BARO_STATUS;

/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* I2C Device Params */
#define BARO_I2C_ADDR	    ( 0x76 << 1 )	/* 1110110 -> 0x76 */


/* Barometric Pressure Sensor register addresses */
#define BARO_REG_CHIP_ID	( 0x00      )

/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* verifies sensor can be accessed */
BARO_STATUS baro_get_device_id
	(
   	uint8_t* baro_id 
	);


/* gets pressure data from sensor */
BARO_STATUS baro_get_pressure
	(
    void
	);

/* gets temp data from sensor */
BARO_STATUS baro_get_temp
	(
    void
	);

/* converts pressure and temp data into altitude --> do research on formula */
BARO_STATUS baro_get_altitude
	(
    void
	);


#endif /* BARO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
