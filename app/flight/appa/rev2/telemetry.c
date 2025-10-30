/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		telemetry.c                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Application integration for LoRa (wireless) communication.             *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------ 
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*------------------------------------------------------------------------------ 
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "common.h"
#include "commands.h"
#include "telemetry.h"

/*------------------------------------------------------------------------------ 
 Global Variables                                                                     
------------------------------------------------------------------------------*/
extern PRESET_DATA   preset_data;      /* Struct with preset data */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */

/*------------------------------------------------------------------------------ 
 Statics                                                                    
------------------------------------------------------------------------------*/
static void telemetry_build_msg_vehicle_id
    (
    LORA_MESSAGE* msg_buf
    );

static void telemetry_build_msg_dashboard_dump
    (
    LORA_MESSAGE* msg_buf
    );

/*------------------------------------------------------------------------------ 
 Procedures                                                                    
------------------------------------------------------------------------------*/

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_payload                                                  *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Builds a LoRa payload.                                                   *
*                                                                                *
*********************************************************************************/
void telemetry_build_payload
    (
    LORA_MESSAGE* msg_buf,
    LORA_MESSAGE_TYPES message_type
    )
{
/*------------------------------------------------------------------------------ 
 Construct Header                                                                    
------------------------------------------------------------------------------*/
memset(msg_buf, 0, LORA_MESSAGE_SIZE);
get_uid( &(msg_buf->header.uid) );
msg_buf->header.mid = message_type;

/*------------------------------------------------------------------------------ 
 Build Payload
------------------------------------------------------------------------------*/
switch( message_type )
    {
    case LORA_MSG_VEHICLE_ID:
        {
        telemetry_build_msg_vehicle_id(msg_buf);
        break;
        }
    case LORA_MSG_DASHBOARD_DATA:
        {
        telemetry_build_msg_dashboard_dump(msg_buf);
        break;
        }
    default:
        {
        error_fail_fast( ERROR_RECORD_FLIGHT_EVENTS_ERROR );
        break;
        }
    }

} /* telemetry_build_payload */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_msg_vehicle_id                                           *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Build the vehicle id payload. Assume header is filled by caller.         *
*                                                                                *
*********************************************************************************/
static void telemetry_build_msg_vehicle_id
    (
    LORA_MESSAGE* msg_buf
    )
{
/* hardware & firmware identifiers */
msg_buf->payload.vehicle_id.hw_opcode = PING_RESPONSE_CODE;
msg_buf->payload.vehicle_id.fw_opcode = FIRMWARE_APPA;

/* version string */
msg_buf->payload.vehicle_id.version |= ( VERSION_HARDWARE << 24 );
msg_buf->payload.vehicle_id.version |= ( VERSION_FIRMWARE_MAJOR << 16 );
msg_buf->payload.vehicle_id.version |= ( VERSION_FIRMWARE_PATCH << 8 );
msg_buf->payload.vehicle_id.version |= ( VERSION_PRERELEASE_NUMBER );

/* flight id (not yet implemented) */
strncpy( msg_buf->payload.vehicle_id.flight_id, "AVIONICS_TEST", 16 );

} /* telemetry_build_msg_vehicle_id */


/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		telemetry_build_msg_dashboard_dump                                       *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Build the dashboard dump payload. Assume header is filled by caller.     *
*                                                                                *
*********************************************************************************/
static void telemetry_build_msg_dashboard_dump
    (
    LORA_MESSAGE* msg_buf
    )
{
/* wrapper */
dashboard_construct_dump( &(msg_buf->payload.dashboard_dump.data) );

} /* telemetry_build_msg_dashboard_dump */
