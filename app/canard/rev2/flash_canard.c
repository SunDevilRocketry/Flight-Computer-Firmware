/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		       flash_canard.c                                                     *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		       Contains all flash special functions for canard application     *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "usb.h"
#include "string.h"

/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/
extern IMU_OFFSET imu_offset;
extern BARO_PRESET baro_preset;
extern SERVO_PRESET servo_preset;
extern SENSOR_DATA sensor_data;

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		store_frame                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Store a frame of flight computer data in flash                         *
*                                                                              *
*******************************************************************************/
FLASH_STATUS store_frame 
	(
	HFLASH_BUFFER* pflash_handle,
	SENSOR_DATA*   sensor_data_ptr,
	uint32_t       time
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t      buffer[DEF_FLASH_BUFFER_SIZE];   /* Sensor data in byte form */
FLASH_STATUS flash_status; /* Flash API status code    */

PRESET_DATA preset_data = {imu_offset, baro_preset, servo_preset};

/*------------------------------------------------------------------------------
 Store Data 
------------------------------------------------------------------------------*/
uint8_t save_bit = 1;
/* Put data into buffer for flash write */
memcpy( &buffer[0], &save_bit, sizeof( uint8_t ) );
memcpy( &buffer[2], &preset_data, sizeof( PRESET_DATA ) );
memcpy( &buffer[36], &time          , sizeof( uint32_t    ) );
memcpy( &buffer[40], sensor_data_ptr, sizeof( SENSOR_DATA ) );

/* Set buffer pointer */
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = DEF_FLASH_BUFFER_SIZE;

/* Write to flash */
flash_status = flash_write( pflash_handle );

/* Return status code */
return flash_status;

} /* store_frame */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		read_current_PID                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read PID prestored in the Flash memory                         			*
*                                                                              *
*******************************************************************************/
FLASH_STATUS read_preset(
	HFLASH_BUFFER* pflash_handle
	)
{
	pflash_handle->address = 0;
	PRESET_DATA preset_data;

	// Look for save bit
	while (1){
		FLASH_STATUS flash_status = flash_read(pflash_handle, DEF_FLASH_BUFFER_SIZE);
		if (flash_status != FLASH_OK)
			{
				return FLASH_FAIL;
			}
		if (pflash_handle->pbuffer[0] == 1){
			break;
		}
		pflash_handle->address += DEF_FLASH_BUFFER_SIZE;

		if (pflash_handle->address > FLASH_MAX_ADDR) {
			// save_bit not found, proceed with default settings
			return FLASH_OK;
		}
	}
	memcpy(&preset_data, &pflash_handle->pbuffer[2], sizeof(PRESET_DATA));
	
	imu_offset = preset_data.imu_offset;
	servo_preset = preset_data.servo_preset;
	baro_preset = preset_data.baro_preset;
	
	return FLASH_OK;
} /* read_preset */
