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


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* verifies sensor can be accessed */
void baro_get_device_id
	(
    void
	);


/* gets pressure data from sensor */
void baro_get_pressure
	(
    void
	);

/* gets temp data from sensor */
void baro_get_temp
	(
    void
	);

/* converts pressure and temp data into altitude --> do research on formula */
void baro_get_altitude
	(
    void
	);


#endif /* BARO_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
