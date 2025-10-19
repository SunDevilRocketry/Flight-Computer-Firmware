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

/*------------------------------------------------------------------------------ 
 Global Variables                                                                     
------------------------------------------------------------------------------*/
extern PRESET_DATA   preset_data;      /* Struct with preset data */
extern SENSOR_DATA   sensor_data;      /* Struct with all sensor */

/*------------------------------------------------------------------------------ 
 Statics                                                                    
------------------------------------------------------------------------------*/


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
    LORA_PAYLOAD* payload_buf
    )
{
/*------------------------------------------------------------------------------ 
 Local Variables                                                                    
------------------------------------------------------------------------------*/
char callsign[6] = "NAUTLS";

/*------------------------------------------------------------------------------ 
 Construct Header                                                                    
------------------------------------------------------------------------------*/

/* Board identifiers */
memset( payload_buf, 0, sizeof( LORA_PAYLOAD ) );
get_uid(&(payload_buf->uid) );
payload_buf->hw_opcode = PING_RESPONSE_CODE;
payload_buf->fw_opcode = FIRMWARE_APPA;

/* Version number */
payload_buf->version |= ( VERSION_HARDWARE << 24 );
payload_buf->version |= ( VERSION_FIRMWARE_MAJOR << 16 );
payload_buf->version |= ( VERSION_FIRMWARE_PATCH << 8 );
payload_buf->version |= ( VERSION_PRERELEASE_NUMBER );

/* ID */
memcpy(payload_buf->flight_id, callsign, 6);

/*------------------------------------------------------------------------------ 
 Construct Data Dump                                                                    
------------------------------------------------------------------------------*/

/* CURRENTLY NOT READY. EXPECT 72 BYTES */

/*------------------------------------------------------------------------------ 
 Sign and Verify                                                                   
------------------------------------------------------------------------------*/

/* CURRENTLY NOT READY. EXPECT 32 BYTES */

} /* telemetry_build_payload */
