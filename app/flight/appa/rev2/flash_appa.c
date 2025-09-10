/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		       flash_logger.c                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		       Contains all flash special functions for APPA            	   *
*			   application.													   *
* 																			   *
* CRITICALITY:																   *
*			   FQ - Flight Qualified    									   *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdlib.h>
#include "main.h"
#include "usb.h"
#include "string.h"
#include "led.h"
#include "imu.h"
#include "sdr_error.h"
#include "buzzer.h"

/*------------------------------------------------------------------------------
 Instantiations                                                                  
------------------------------------------------------------------------------*/
uint8_t sensor_frame_size = 0;
uint8_t num_preset_frames = 1;

extern IMU_OFFSET imu_offset;
extern BARO_PRESET baro_preset;
extern SENSOR_DATA sensor_data;
extern CONFIG_SETTINGS_TYPE config_settings;
extern FLIGHT_COMP_STATE_TYPE flight_computer_state;
extern float feedback;

/*------------------------------------------------------------------------------
 Local Procedures                                                                  
------------------------------------------------------------------------------*/
static void set_default_configs
	(
	void
	);


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
FLASH_STATUS flash_status; 		/* Flash API status code    */

/*------------------------------------------------------------------------------
 Initialize sensor data 
------------------------------------------------------------------------------*/
if (!sensor_frame_size)
	{
	sensor_frame_size_init();
	*address = sensor_frame_size * num_preset_frames;
	}

uint8_t buffer[sensor_frame_size];   		/* Sensor data in byte form */
flash_status = get_sensor_frame(sensor_data_ptr, buffer, time);

/* Skip logging if error detected */
if( flash_status != FLASH_OK )
	{
	return flash_status;
	}

/*------------------------------------------------------------------------------
 Store Data 
------------------------------------------------------------------------------*/
uint8_t save_bit = 1;
/* Put data into buffer for flash write */
memcpy( &buffer[0], &save_bit, sizeof( uint8_t ) );
memcpy( &buffer[1], &flight_computer_state, sizeof( FLIGHT_COMP_STATE_TYPE ) );
memcpy( &buffer[2], &time          , sizeof( uint32_t    ) );

/* Set buffer pointer */
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = sensor_frame_size;
pflash_handle->address = *address;

/* Write to flash */
flash_status = flash_write( pflash_handle );

/* Update the address pointer */
*address = pflash_handle->address + pflash_handle->num_bytes;

/* Return status code */
return flash_status;

} /* store_frame */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		read_preset                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Read configuration data from the flash memory                          *
*                                                                              *
*******************************************************************************/
FLASH_STATUS read_preset
	(
	HFLASH_BUFFER* pflash_handle,
	PRESET_DATA*   preset_data_ptr,
	uint32_t*	   address
	)
{
uint8_t      buffer[ sizeof( PRESET_DATA ) + 2 ];   /* Sensor data in byte form */
memset(buffer, 0, sizeof( PRESET_DATA ) + 2 );
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->address = 0;
pflash_handle->num_bytes = sizeof( PRESET_DATA );
// Look for save bit
while (1){ /* could change to a for loop i < PRESET_WRITE_REPEATS */ 
	while( flash_is_flash_busy() == FLASH_BUSY )
		{
		led_set_color(LED_YELLOW);
		}

	FLASH_STATUS flash_status = flash_read( pflash_handle, sizeof( PRESET_DATA ) );
	if (flash_status != FLASH_OK)
		{
			return FLASH_FAIL;
		}
	if (pflash_handle->pbuffer[0] == 1){
		break;
	}
	pflash_handle->address += sizeof( PRESET_DATA );
	if (pflash_handle->address + (sizeof( PRESET_DATA )) > FLASH_MAX_ADDR) {
		// save_bit not found, proceed with default settings
		pflash_handle->address = 0;
		set_default_configs();
		return FLASH_PRESET_NOT_FOUND;
	}
}

memcpy(preset_data_ptr, &(buffer)[2], sizeof(PRESET_DATA));

config_settings = preset_data_ptr->config_settings;
imu_offset = preset_data_ptr->imu_offset;
baro_preset = preset_data_ptr->baro_preset;

*address = 0;

return FLASH_OK;

} /* read_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		write_preset	                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Store PID data and offsets in flash. 34 bytes.     	                   *
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
Init
------------------------------------------------------------------------------*/
if (!sensor_frame_size)
	{
	sensor_frame_size_init();
	}

/*------------------------------------------------------------------------------
Local variables 
------------------------------------------------------------------------------*/
uint8_t      buffer[ sizeof( PRESET_DATA) + 2 ];   /* Sensor data in byte form */
FLASH_STATUS flash_status; /* Flash API status code    */
memset( &buffer, 0, sizeof( PRESET_DATA) + 2 );

/*------------------------------------------------------------------------------
 Store Data 
------------------------------------------------------------------------------*/
/* 
 Erase old preset data by erasing the first 4KB sector
*/
flash_status = flash_block_erase( FLASH_BLOCK_4K, FLASH_BLOCK_0 );

while( flash_is_flash_busy() == FLASH_BUSY )
	{
	led_set_color(LED_YELLOW);
	}

uint8_t save_bit = 1;

/* Put data into buffer for flash write */
memcpy( &buffer[0], &save_bit, sizeof( uint8_t ) );
memcpy( &buffer[2], preset_data_ptr, sizeof( PRESET_DATA ) );

/* Write to flash */
pflash_handle->address = 0;
pflash_handle->pbuffer   = &buffer[0];
pflash_handle->num_bytes = sensor_frame_size;
flash_status = flash_write( pflash_handle );

/* Update the address */
*address = sensor_frame_size;

/* Return status code */
return flash_status;

} /* write_preset */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		flash_erase_preserve_preset	                                           *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Erases flash while saving the stored preset values.     	           *
*                                                                              *
*******************************************************************************/
FLASH_STATUS flash_erase_preserve_preset
	(
	HFLASH_BUFFER* pflash_handle,
	uint32_t* address
	)
{

if (!sensor_frame_size)
	{
	sensor_frame_size_init();
	}

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


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		get_sensor_frame                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Gets the contents of a sensor frame based on config data.              *
*                                                                              *
*******************************************************************************/
FLASH_STATUS get_sensor_frame
	(
	SENSOR_DATA* sensor_data_ptr, /* i: sensor data struct */
	uint8_t* buffer, /* o: sensor frame */
	uint32_t time 	 /* i: frame timestamp */
	)
{
/* Local Variables */
uint8_t idx = 0; /* current index in the buffer */

/* Clear the allocated memory */
memset(buffer, 0, sensor_frame_size);
buffer[0] = 1; /* save bit */
buffer[1] = flight_computer_state;
buffer[2] = time;
idx = 6;

if ( preset_data.config_settings.enabled_data & STORE_RAW )
	{
	memcpy( &buffer[idx], /* only grabs raw data and leaves off state estimations */
			&sensor_data_ptr->imu_data, 
			(10 * sizeof(uint16_t))  );
	idx += (10 * sizeof(uint16_t));
	memcpy( &buffer[idx], &sensor_data_ptr->baro_pressure, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->baro_temp, sizeof(float));
	idx += 4;
	}

if ( preset_data.config_settings.enabled_data & STORE_CONV )
	{
	memcpy( &buffer[idx],
			&sensor_data_ptr->imu_data.imu_converted,
			sizeof( IMU_CONVERTED ));
	idx += sizeof( IMU_CONVERTED );
	}

if ( preset_data.config_settings.enabled_data & STORE_STATE_ESTIM )
	{
	memcpy( &buffer[idx],
			&sensor_data_ptr->imu_data.state_estimate,
			sizeof( STATE_ESTIMATION ));
	idx += sizeof( STATE_ESTIMATION );
	memcpy( &buffer[idx], &sensor_data_ptr->baro_alt, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->baro_velo, sizeof(float));
	idx += 4;
	}

if ( preset_data.config_settings.enabled_data & STORE_GPS )
	{
	memcpy( &buffer[idx], &sensor_data_ptr->gps_altitude_ft, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_speed_kmh, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_utc_time, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_dec_longitude, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_dec_latitude, sizeof(float));
	idx += 4;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_ns, sizeof(char));
	idx++;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_ew, sizeof(char));
	idx++;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_gll_status, sizeof(char));
	idx++;
	memcpy( &buffer[idx], &sensor_data_ptr->gps_rmc_status, sizeof(char));
	idx++;
	}

if ( preset_data.config_settings.enabled_data & STORE_CANARD_DATA )
	{
	memcpy( &buffer[idx], &feedback, sizeof(float));
	idx += 4;
	}

/* Validity check: Verify the allocated size matches the final index*/
if (sensor_frame_size == idx)
	{
	return FLASH_OK;
	}
else
	{
	return FLASH_SENSOR_RETRIEVE_ERROR;
	}

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		sensor_frame_size_init	                                           	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Calculates the size of a sensor frame from configs.     	           *
*                                                                              *
*******************************************************************************/
void sensor_frame_size_init
	(
	void
	)
{
/* Local Variables */
uint8_t size = 0; /* value to return; size of buffer */

/* Allocate the required memory */
size += 6; /* space for save bit, FC state, and time. */

if ( preset_data.config_settings.enabled_data & STORE_RAW )
	{
	size += 10 * sizeof( uint16_t ); 	/* IMU raw data 	*/
	size += 2 * sizeof( float ); 		/* baro raw data	*/
	}

if ( preset_data.config_settings.enabled_data & STORE_CONV )
	{
	size += sizeof( IMU_CONVERTED );
	}

if ( preset_data.config_settings.enabled_data & STORE_STATE_ESTIM )
	{
	size += sizeof( STATE_ESTIMATION );
	size += 2 * sizeof( float ); /* baro alt/velo */
	}

if ( preset_data.config_settings.enabled_data & STORE_GPS )
	{
	size += 5 * sizeof( float );
	size += 4 * sizeof( char );
	}

if ( preset_data.config_settings.enabled_data & STORE_CANARD_DATA )
	{
	size += 1 * sizeof( float ); /* feedback */
	}

sensor_frame_size = size;
num_preset_frames = 1;
while ( size < sizeof( PRESET_DATA ) )
	{
	size += sensor_frame_size;
	num_preset_frames++;
	}

}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		set_default_configs	                                           	   	   *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Sets the configuration struct to default values (save bit not found)   *
*                                                                              *
*******************************************************************************/
static void set_default_configs
	(
	void
	) 
{
led_set_color( LED_CYAN );
buzzer_multi_beeps( 500, 500, 3 );
preset_data.config_settings.enabled_features = 0b11100001; /* launch detect, dual deploy, data logging */
preset_data.config_settings.enabled_data = 0b11111111; 	   /* all data enabled */
preset_data.config_settings.sensor_calibration_samples = 1000;		/* unitless */
preset_data.config_settings.launch_detect_timeout 	   = 30000; 		/* unit: ms */
preset_data.config_settings.launch_detect_accel_threshold = 2;		/* unit: g	*/
preset_data.config_settings.launch_detect_accel_samples	  = 5;		/* unitless */
preset_data.config_settings.launch_detect_baro_threshold  = 300;	/* unit: Pa */
preset_data.config_settings.launch_detect_baro_samples	  = 5;		/* unitless */
preset_data.config_settings.control_delay_after_launch	  = 4000;	/* unit: ms */
preset_data.config_settings.roll_control_constant_p = 0.0f; /* active control disabled */
preset_data.config_settings.roll_control_constant_i = 0.0f; /* active control disabled */
preset_data.config_settings.roll_control_constant_d = 0.0f; /* active control disabled */
preset_data.config_settings.pitch_yaw_control_constant_p = 0.0f; /* active control disabled */
preset_data.config_settings.pitch_yaw_control_constant_i = 0.0f; /* active control disabled */
preset_data.config_settings.pitch_yaw_control_constant_d = 0.0f; /* active control disabled */
preset_data.config_settings.control_max_deflection_angle = 0;	/* active control disabled */
preset_data.config_settings.minimum_time_for_frame = 0;			/* unit: ms */
}