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
#include "led.h"

/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/
extern IMU_OFFSET 	imu_offset;
extern BARO_PRESET 	baro_preset;
extern SERVO_PRESET servo_preset;
extern SENSOR_DATA 	sensor_data;
extern uint8_t 	   	acc_detect_flag;
extern float	   	feedback;


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
	uint32_t       time,
	uint32_t*	   address
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t      buffer[DEF_FLASH_BUFFER_SIZE];   /* Sensor data in byte form */
FLASH_STATUS flash_status; /* Flash API status code    */

/*------------------------------------------------------------------------------
 Store Data 
------------------------------------------------------------------------------*/
uint8_t save_bit = 1;
/* Put data into buffer for flash write */
memcpy( &buffer[0], &save_bit, sizeof( uint8_t ) );
memcpy( &buffer[1], &acc_detect_flag, sizeof( uint8_t ) );
memcpy( &buffer[2], &time          , sizeof( uint32_t    ) );
memcpy( &buffer[6], sensor_data_ptr, sizeof( SENSOR_DATA ) );
/* Add canard specific data to the end of the frame */
memcpy( &buffer[DEF_FLASH_BUFFER_SIZE - 4], &feedback, sizeof( float ) );

/* Set buffer pointer */
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = DEF_FLASH_BUFFER_SIZE;
pflash_handle->address = *address;

/* Write to flash */
flash_status = flash_write( pflash_handle );

/* Return status code */
return flash_status;

} /* store_frame */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		read_preset                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read configuration data in the flash memory                         			*
*                                                                              *
*******************************************************************************/
FLASH_STATUS read_preset(
	HFLASH_BUFFER* pflash_handle,
	PRESET_DATA*   preset_data_ptr,
	uint32_t*	   address
	)
{
uint8_t      buffer[DEF_FLASH_BUFFER_SIZE];   /* Sensor data in byte form */
memset(buffer, 0, DEF_FLASH_BUFFER_SIZE);
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->address = 0;
pflash_handle->num_bytes = DEF_FLASH_BUFFER_SIZE;
// Look for save bit
while (1) /* could add a timeout (at which we set flash_status = flash busy) */ 
	{
	while( flash_is_flash_busy() == FLASH_BUSY )
		{
		led_set_color(LED_YELLOW);
		}

	FLASH_STATUS flash_status = flash_read(pflash_handle, DEF_FLASH_BUFFER_SIZE);
	if (flash_status != FLASH_OK)
		{
			return FLASH_FAIL;
		}
	if (pflash_handle->pbuffer[0] == 1)
		{
		break;
		}
	pflash_handle->address += DEF_FLASH_BUFFER_SIZE;
	if (pflash_handle->address + DEF_FLASH_BUFFER_SIZE > FLASH_MAX_ADDR) 
		{
		// save_bit not found, proceed with default settings
		pflash_handle->address = 0;
		return FLASH_PRESET_NOT_FOUND;
		}
	}

memcpy(preset_data_ptr, &(buffer)[2], sizeof(PRESET_DATA));

imu_offset = preset_data_ptr->imu_offset;
servo_preset = preset_data_ptr->servo_preset;
baro_preset = preset_data_ptr->baro_preset;

*address = pflash_handle->address + DEF_FLASH_BUFFER_SIZE;

return FLASH_OK;

} /* read_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		write_preset	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Store PID data and offsets in flash. 40 bytes.     	                   *
*                                                                              *
*******************************************************************************/
FLASH_STATUS write_preset 
	(
	HFLASH_BUFFER* pflash_handle,
	PRESET_DATA*   preset_data_ptr,
	uint32_t* 	   address
	)
{
/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t      buffer[DEF_FLASH_BUFFER_SIZE];   /* Sensor data in byte form */
FLASH_STATUS flash_status; /* Flash API status code    */

/* Erase old preset data by erasing the first 4KB sector */
flash_status = flash_block_erase( FLASH_BLOCK_4K, FLASH_BLOCK_0 );

/*------------------------------------------------------------------------------
 Store Data 
------------------------------------------------------------------------------*/
while( flash_is_flash_busy() == FLASH_BUSY )
	{
	led_set_color(LED_YELLOW);
	}

uint8_t save_bit = 1;
uint8_t empty_byte = 0;

/* Put data into buffer for flash write */
memcpy( &buffer[0], &save_bit, sizeof( uint8_t ) );
memcpy( &buffer[1], &empty_byte, sizeof(uint8_t));
// memcpy( &buffer[2], preset_data_ptr, sizeof( PRESET_DATA ) );
memcpy( &buffer[2], preset_data_ptr, sizeof( PRESET_DATA ) );

/* Write to flash */
pflash_handle->address = 0;
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = DEF_FLASH_BUFFER_SIZE;
flash_status = flash_write( pflash_handle );

// /* Update the address pointer to first byte of first sensor frame
//    and zero out the memory between */
// uint8_t length_pad[DEF_FLASH_BUFFER_SIZE - DEF_PRESET_BUFFER_SIZE];
// memset(length_pad, 0, sizeof(length_pad));
// pflash_handle->address = DEF_PRESET_BUFFER_SIZE;
// pflash_handle->pbuffer = &length_pad[0];
// pflash_handle->num_bytes = sizeof(length_pad);
// flash_status = flash_write( pflash_handle );

/* Update the address */
*address = DEF_FLASH_BUFFER_SIZE;

/* Return status code */
return flash_status;

} /* write_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_erase_preserve_preset	                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Erase all of flash, then write the preset data back  	               *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_erase_preserve_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* address
	)
{
/* Read the presets */
PRESET_DATA presets;
*address = 0;
FLASH_STATUS status = read_preset( pflash_handle, &presets, address );
if ( status != FLASH_OK )
	{
	return status;
	}

/* Erase flash */
status = flash_erase( pflash_handle );
if ( status != FLASH_OK )
	{
	return status;
	}

/* Write the presets back */
*address = 0;
status = write_preset( pflash_handle, &presets, address );
return status;

} /* flash_erase_preserve_preset */
