/*******************************************************************************
*
* FILE: 
* 		baro.h
*
* DESCRIPTION: 
* 		Contains API functions for the barometric pressure sensor
*
*******************************************************************************/


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
