#ifndef STUBS /* recursive inclusion guard */

#include "error_sdr.h"

/* globals */
extern bool was_gps_enabled;
extern bool is_apogee_detected;
extern uint16_t preset_preserving_flash_erase_calls;
extern uint16_t flash_busy_calls;
extern uint16_t flash_busy_counts;

/* functions */
void stubs_reset();
void set_return_ign_deploy_main
	(
	IGN_STATUS new_status[3]
	);
void set_return_ign_deploy_drogue
	(
	IGN_STATUS new_status[3]
	);
unsigned int get_num_calls_ign_deploy_main();
unsigned int get_num_calls_ign_deploy_drogue();
void set_return_HAL_GetTick(uint32_t ret);
unsigned int get_num_calls_HAL_GetTick();
SERVO_PRESET get_servo_angles_struct();
void set_error_callback( void ( *input_callback )( ERROR_CODE ) );
void set_return_sensor_dump( SENSOR_STATUS return_val );
void set_return_launch_detection( bool expected );
#endif