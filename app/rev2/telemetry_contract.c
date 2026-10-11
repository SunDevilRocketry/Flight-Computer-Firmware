/**
  ******************************************************************************
  * @file           : telemetry_contract.c
  * @brief          : Implements the transmission contracts from telemetry.h
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

/*------------------------------------------------------------------------------ 
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*------------------------------------------------------------------------------ 
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "math_sdr.h"
#include "commands.h"
#include "error_sdr.h"
#include "debug_sdr.h"
#include "telemetry.h"
#include "lora.h"
#include "usb.h"
#include "led.h"
#include "buzzer.h"
#include "isa.h"

/*------------------------------------------------------------------------------ 
 Global Variables                                                                     
------------------------------------------------------------------------------*/
extern PRESET_DATA   preset_data;      /* Struct with preset data   */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor    */

static uint32_t      message_idx = 0;

/*------------------------------------------------------------------------------ 
 Statics                                                                    
------------------------------------------------------------------------------*/

static void telemetry_build_msg_vehicle_id
    (
    TELEMETRY_MESSAGE* msg_buf
    );

static void telemetry_build_msg_calibration
    (
    TELEMETRY_MESSAGE* msg_buf
    );

static void telemetry_build_msg_dashboard_dump
    (
    TELEMETRY_MESSAGE* msg_buf
    );

static void dashboard_construct_dump
    (
    DASHBOARD_DUMP_TYPE* dump_buffer_ptr
    );

/*------------------------------------------------------------------------------ 
 Contract Implementations                                                              
------------------------------------------------------------------------------*/

/**
 * @brief Initialize the telemetry subsystem and begin transmitting
 * 
 * @return LORA_STATUS The current status of the LoRa/Telemetry system.
 */
LORA_STATUS telemetry_init
    (
    void
    ) 
{
/* Local variables */
LORA_STATUS lora_status = LORA_OK;

/* Enable LORA */
if ( preset_data.config_settings.enabled_features & WIRELESS_TRANSMISSION_ENABLED )
    {
    if( !lora_is_lora_initialized() )
        {
        lora_status = lora_configure( &preset_data.lora_preset );
        debug_assert( lora_status == LORA_OK, ERROR_LORA_INIT_ERROR );
        }
    
    /* If the modem fails to configure, disable TX in RAM (but do not write back to flash) */
    if( lora_status != LORA_OK )
        {
        preset_data.config_settings.enabled_features &= ~WIRELESS_TRANSMISSION_ENABLED;

        /* Give an indication */
        led_set_color(LED_RED);
        buzzer_multi_beeps(400, 200, 3);
        led_set_color(LED_YELLOW);
        }

    lora_fsm_set_mode( LORA_ASYNC_TX );
    }

return lora_status;

} /* telemetry_init */

/**
  * @brief Sends the data required by the dashboard.
  *
  * @return USB transmission status.
  */
USB_STATUS dashboard_dump
    (
    void
    )
{
/*------------------------------------------------------------------------------
 Local variables                                                                     
------------------------------------------------------------------------------*/
DASHBOARD_DUMP_TYPE buffer;
SENSOR_STATUS sensor_status = SENSOR_OK;

sensor_status = sensor_dump( &sensor_data );

if ( !( sensor_status == SENSOR_OK ) )
    {
    return USB_FAIL;
    }

dashboard_construct_dump( &buffer );

return usb_transmit( &buffer, 
                        DASHBOARD_DUMP_SIZE, 
                        HAL_SENSOR_TIMEOUT /* more forgiving HW timeout */ );

} /* dashboard_dump */


/**
  * @brief Selects and constructs the next scheduled telemetry message.
  * @param payload Telemetry message buffer to fill.
  */
void telemetry_get_next_message
    (
    TELEMETRY_MESSAGE* payload /* o: constructed telemetry message */
    )
{
TELEMETRY_MESSAGE_TYPES msg_type;

/* Determine which payload to send */
if( ( message_idx % 3 == 0 )
    && ( get_fc_state() == FC_STATE_LAUNCH_DETECT ) )
    {
    msg_type = TELEMETRY_MSG_VEHICLE_ID;
    }
else if( ( message_idx % 3 == 1 )
    && ( get_fc_state() == FC_STATE_LAUNCH_DETECT ) )
    {
    msg_type = TELEMETRY_MSG_CALIBRATION;
    }
else
    {
    msg_type = TELEMETRY_MSG_DASHBOARD_DATA;
    }
telemetry_build_payload(payload, msg_type);
message_idx++;

} /* telemetry_get_next_message */


/**
  * @brief Builds the requested telemetry payload and fills its message header.
  * @param msg_buf Telemetry message buffer to fill.
  * @param message_type Type of telemetry message to build.
  */
void telemetry_build_payload
    (
    TELEMETRY_MESSAGE*       msg_buf,      /* o: buffer passed by caller        */
    TELEMETRY_MESSAGE_TYPES  message_type  /* i: what kind of message           */
    )
{
/*------------------------------------------------------------------------------ 
 Construct Header                                                                    
------------------------------------------------------------------------------*/
memset(msg_buf, 0, TELEMETRY_MESSAGE_SIZE);
msg_buf->header.mid = message_type;
msg_buf->header.timestamp = HAL_GetTick();

/*------------------------------------------------------------------------------ 
 Build Payload
------------------------------------------------------------------------------*/
switch( message_type )
    {
    case TELEMETRY_MSG_VEHICLE_ID:
        {
        telemetry_build_msg_vehicle_id(msg_buf);
        break;
        }
    case TELEMETRY_MSG_DASHBOARD_DATA:
        {
        telemetry_build_msg_dashboard_dump(msg_buf);
        break;
        }
    case TELEMETRY_MSG_CALIBRATION:
        {
        telemetry_build_msg_calibration(msg_buf);
        break;
        }
    default:
        {
        /** In order to support projects with safety critical 
          * requirements, mod does not throw fail-fast errors.
          * Thus, this should nop on release builds.
          * 
          * This ensures that the library will only fail fast in
          * the event of a hardfault and enables it to offload
          * FHA requirements to the project that integrates it,
          * if needed.
          * 
          * GCOVR_EXCL_START
          *
          * A coverage hole is reported here because the debug-mode
          * emulator tests have an actual function call here while
          * release does not. This branch *is* tested in the release
          * configuration.
          */
        debug_assert( false, ERROR_RECORD_FLIGHT_EVENTS_ERROR );
        break;
        /* GCOVR_EXCL_STOP */
        }
    }

/*------------------------------------------------------------------------------ 
 Encrypt Message
------------------------------------------------------------------------------*/
/* not yet ready */

// ETS Temp: This may not be done by v2.6.0. I'm happy to commit to dropping
// encryption for this release since our comms are one-way
// flight critical --> non-critical, so the flight critical element is never
// receiving outside data in-flight.

} /* telemetry_build_payload */

/*------------------------------------------------------------------------------ 
 State Functions                                                                    
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------ 
 Telemetry Message Constructors                                                                    
------------------------------------------------------------------------------*/

/**
  * @brief Builds a payload containing the vehicle identification and version.
  * @param msg_buf Telemetry message buffer to fill.
  */
static void telemetry_build_msg_vehicle_id
    (
    TELEMETRY_MESSAGE* msg_buf
    )
{
/* hardware & firmware identifiers */
msg_buf->payload.vehicle_id.hw_opcode = PING_RESPONSE_CODE;
msg_buf->payload.vehicle_id.fw_opcode = FIRMWARE_APPA;

/* STM32 UID */
msg_buf->payload.vehicle_id.uid[0] = HAL_GetUIDw0();
msg_buf->payload.vehicle_id.uid[1] = HAL_GetUIDw1();
msg_buf->payload.vehicle_id.uid[2] = HAL_GetUIDw2();

/* version string */
msg_buf->payload.vehicle_id.version |= ( VERSION_HARDWARE << 24 );
msg_buf->payload.vehicle_id.version |= ( VERSION_FIRMWARE_MAJOR << 16 );
msg_buf->payload.vehicle_id.version |= ( VERSION_FIRMWARE_PATCH << 8 );
msg_buf->payload.vehicle_id.version |= ( VERSION_PRERELEASE_NUMBER );

/* flight id (not yet implemented) */
strncpy( msg_buf->payload.vehicle_id.flight_id, "AVIONICS_TEST", 16 );

} /* telemetry_build_msg_vehicle_id */


/**
  * @brief Builds a payload containing calibration data and QFE elevation.
  * @param msg_buf Telemetry message buffer to fill.
  */
static void telemetry_build_msg_calibration
    (
    TELEMETRY_MESSAGE* msg_buf
    )
{
/*------------------------------------------------------------------------------ 
 Local Variables                                    
------------------------------------------------------------------------------*/
SENSOR_DATA qfe_sensor_data;

memset( &qfe_sensor_data, 0, sizeof( SENSOR_DATA ) );

/*------------------------------------------------------------------------------ 
 Construct known elements from preset data                                      
------------------------------------------------------------------------------*/
memcpy( &(msg_buf->payload.calibration.imu_offset), &(preset_data.imu_offset), sizeof( IMU_OFFSET ) );
memcpy( &(msg_buf->payload.calibration.baro_preset), &(preset_data.baro_preset), sizeof( BARO_PRESET ) );
memcpy( &(msg_buf->payload.calibration.servo_preset), &(preset_data.servo_preset), sizeof( SERVO_PRESET ) );

/*------------------------------------------------------------------------------ 
 Determine QFE reference elevation

 Counterintuitively, we want QNH for this. This way, we can subtract it from 
 the QNH altitude to yield a QFE altitude.
------------------------------------------------------------------------------*/
msg_buf->payload.calibration.qfe_elevation = isa_get_altitude_qnh( preset_data.baro_preset.baro_pres, preset_data.baro_preset.baro_temp );

} /* telemetry_build_msg_calibration */


/**
  * @brief Builds a payload containing the current flight state and dashboard data.
  * @param msg_buf Telemetry message buffer to fill.
  */
static void telemetry_build_msg_dashboard_dump
    (
    TELEMETRY_MESSAGE* msg_buf
    )
{
msg_buf->payload.dashboard_dump.fsm_state = get_fc_state();
dashboard_construct_dump( &(msg_buf->payload.dashboard_dump.data) );

} /* telemetry_build_msg_dashboard_dump */


/**
  * @brief Fill the buffer with the dashboard dump.
  *
  * @param dump_buffer_ptr Pointer to the dashboard dump buffer. Must be
  *        DASHBOARD_DUMP_SIZE.
  */
static void dashboard_construct_dump
    (
    DASHBOARD_DUMP_TYPE* dump_buffer_ptr /* must be DASHBOARD_DUMP_SIZE */
    )
{
/* Quats */
dump_buffer_ptr->attitude = sensor_data.state_estimate.attitude;

/* Baro */
dump_buffer_ptr->alt = sensor_data.baro_alt;

/* GPS */
dump_buffer_ptr->longitude = sensor_data.gps_dec_longitude;
dump_buffer_ptr->latitude = sensor_data.gps_dec_latitude;

/* Controls */
dump_buffer_ptr->acc_x = sensor_data.imu_converted.accel_x;
dump_buffer_ptr->roll_rate = sensor_data.state_estimate.roll_rate;

} /* dashboard_construct_dump */
