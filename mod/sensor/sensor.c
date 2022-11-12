/*******************************************************************************
*
* FILE: 
* 		sensor.c
*
* DESCRIPTION: 
* 		Contains functions to interface between sdec terminal commands and SDR
*       sensor APIs
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <string.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "imu.h"
#include "sensor.h"

/*------------------------------------------------------------------------------
 Global Variables 
------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart6; /* USB UART handler struct        */


/*------------------------------------------------------------------------------
 Public procedures 
------------------------------------------------------------------------------*/

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_cmd_execute                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Executes a sensor subcommand                                           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_cmd_execute 
	(
    uint8_t subcommand 
    )
{

/*------------------------------------------------------------------------------
 Local Variables  
------------------------------------------------------------------------------*/
SENSOR_STATUS sensor_subcmd_status;                 /* Status indicating if 
                                                       subcommand function 
                                                       returned properly      */
SENSOR_DATA   sensor_data;                           /* Struct with all sensor 
                                                        data                  */
uint8_t       sensor_data_bytes[ SENSOR_DATA_SIZE ]; /* Byte array with sensor 
                                                       readouts               */
const uint8_t num_sensor_bytes = SENSOR_DATA_SIZE;

/*------------------------------------------------------------------------------
 Initializations  
------------------------------------------------------------------------------*/
memset ( &sensor_data_bytes[0], 0, sizeof( sensor_data_bytes ) );
memset ( &sensor_data         , 0, sizeof( sensor_data       ) );


/*------------------------------------------------------------------------------
 Execute Sensor Subcommand 
------------------------------------------------------------------------------*/
switch ( subcommand )
	{

	/* Poll Sensors continuously */
    case SENSOR_POLL_CODE:
		{
		// TODO: Implement sensor poll function 
		return ( SENSOR_UNSUPPORTED_OP );
        } /* SENSOR_POLL_CODE */ 

	/* Poll sensors once and dump data on terminal */
	case SENSOR_DUMP_CODE: 
		{
		/* Tell the PC how many bytes to expect */
		HAL_UART_Transmit( &huart6,
                           &num_sensor_bytes,
                           sizeof( num_sensor_bytes ), 
                           HAL_DEFAULT_TIMEOUT );

		/* Get the sensor readings */
	    sensor_subcmd_status = sensor_dump( &sensor_data );	

		/* Convert to byte array */
		memcpy( &(sensor_data_bytes[0]), &sensor_data, sizeof( sensor_data ) );

		/* Transmit sensor readings to PC */
		if ( sensor_subcmd_status == SENSOR_OK )
			{
			// readings_to_bytes( &sensor_readings_bytes[0], 
            //                    &sensor_readings[0] );
			HAL_UART_Transmit( &huart6                    , 
                               &sensor_data_bytes[0]      , 
                               sizeof( sensor_data_bytes ), 
                               HAL_SENSOR_TIMEOUT );
			return ( sensor_subcmd_status );
            }
		else
			{
			/* Sensor readings not recieved */
			return( SENSOR_FAIL );
            }
        } /* SENSOR_DUMP_CODE */

	/* Subcommand not recognized */
	default:
		{
		return ( SENSOR_UNRECOGNIZED_OP );
        }
    }

} /* sensor_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_dump                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       reads from all sensors and fill in the sensor data structure           *
*                                                                              *
*******************************************************************************/
SENSOR_STATUS sensor_dump 
	(
    SENSOR_DATA*        sensor_data_ptr /* Pointer to the sensor data struct should 
                                        be written */ 
    )
{
/*------------------------------------------------------------------------------
 Local variables 
------------------------------------------------------------------------------*/
IMU_STATUS      accel_status;           /* IMU sensor status codes     */       
IMU_STATUS      gyro_status;
IMU_STATUS      mag_status; 
uint16_t        baro_pressure;          /* Baro Sensor Readouts        */
uint16_t        baro_temp;


/*------------------------------------------------------------------------------
 Call sensor API functions 
------------------------------------------------------------------------------*/

/* Poll the IMU sensors */
accel_status = imu_get_accel_xyz( &(sensor_data_ptr->imu_data) ); 
gyro_status  = imu_get_gyro_xyz ( &(sensor_data_ptr->imu_data) );
mag_status   = imu_get_mag_xyz  ( &(sensor_data_ptr->imu_data) );

/* Poll the Baro sensors */
// TODO: Implement the actual get baro values
// Temporary: Set the baro readings to 0
baro_pressure                  = 0;
baro_temp                      = 0;
sensor_data_ptr->baro_pressure = baro_pressure;
sensor_data_ptr->baro_temp     = baro_temp;


/*------------------------------------------------------------------------------
 Set command status from sensor API returns 
------------------------------------------------------------------------------*/
if ( accel_status != IMU_TIMEOUT &&
     gyro_status  != IMU_TIMEOUT &&
     mag_status   != IMU_TIMEOUT  )
    {
        return SENSOR_OK;
    }
    else
    {
        return SENSOR_IMU_FAIL;
    }
} /* sensor_dump */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
