/*******************************************************************************
*
* FILE: 
* 		telemetry.h
*
* DESCRIPTION: 
* 		Definitions for the telemetry data structures.
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TELEM_H
#define __TELEM_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Standard Includes                                                                    
------------------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/*------------------------------------------------------------------------------
 Project Includes  
------------------------------------------------------------------------------*/
#include "common.h"
#include "commands.h"
#include "sensor.h"
#include "main.h"

/*------------------------------------------------------------------------------
 Constants
------------------------------------------------------------------------------*/
#define LORA_INTERNAL_HEADER_SIZE 20U
#define LORA_PAYLOAD_SIZE 76U
#define LORA_MESSAGE_SIZE (LORA_INTERNAL_HEADER_SIZE + LORA_PAYLOAD_SIZE)

/*------------------------------------------------------------------------------
 Typedefs
------------------------------------------------------------------------------*/

typedef enum _LORA_MESSAGE_TYPES
    {
    LORA_MSG_VEHICLE_ID = 0x00000001,
    LORA_MSG_DASHBOARD_DATA = 0x00000002,
    __LORA_MSG_FORCE_32BIT = 0xFFFFFFFF /* used to force this type size to 32 bits */
    } LORA_MESSAGE_TYPES;
    _Static_assert( sizeof(LORA_MESSAGE_TYPES) == 4, "LORA_MESSAGE_TYPES size invalid.");

typedef struct __attribute__((packed)) _LORA_INTERNAL_HEADER_TYPE
    {
    ST_UID_TYPE uid;
    LORA_MESSAGE_TYPES mid;
    uint32_t timestamp;
    } LORA_INTERNAL_HEADER_TYPE;
    _Static_assert( sizeof(LORA_INTERNAL_HEADER_TYPE) == LORA_INTERNAL_HEADER_SIZE, "LORA_INTERNAL_HEADER size invalid.");

typedef struct __attribute((packed)) _LORA_MSG_VEHICLE_ID_TYPE
    {
    uint8_t hw_opcode;
	uint8_t fw_opcode;
	VERSION_INFO_TYPE version;
	char flight_id[16];
    uint8_t explicit_padding[54];
    } LORA_MSG_VEHICLE_ID_TYPE;
    _Static_assert( sizeof(LORA_MSG_VEHICLE_ID_TYPE) == LORA_PAYLOAD_SIZE, "LORA_MSG_VEHICLE_ID_TYPE size invalid.");

typedef struct __attribute__((packed)) _LORA_MSG_DASHBOARD_DUMP_TYPE
    {
    FLIGHT_COMP_STATE_TYPE fsm_state;
    DASHBOARD_DUMP_TYPE data;
    uint8_t explicit_padding[3];
    } LORA_MSG_DASHBOARD_DUMP_TYPE;
    _Static_assert( sizeof(LORA_MSG_DASHBOARD_DUMP_TYPE) == LORA_PAYLOAD_SIZE, "LORA_MSG_DASHBOARD_DUMP_TYPE size invalid.");

/* struct is packed to inhibit padding */
typedef struct __attribute__((packed)) _LORA_MESSAGE
	{
	LORA_INTERNAL_HEADER_TYPE header;
    union _payload
        {
        LORA_MSG_VEHICLE_ID_TYPE vehicle_id;
        LORA_MSG_DASHBOARD_DUMP_TYPE dashboard_dump;
        } payload;
	} LORA_MESSAGE;
	_Static_assert( sizeof(LORA_MESSAGE) == LORA_MESSAGE_SIZE, "LORA_PAYLOAD size invalid.");


/*------------------------------------------------------------------------------
 Global Variables                                             
------------------------------------------------------------------------------*/
extern PRESET_DATA preset_data;
extern SENSOR_DATA sensor_data;
extern uint8_t sensor_frame_size;
extern uint8_t num_preset_frames;

/* Timing (debug) */
#ifdef DEBUG
extern volatile uint32_t debug_previous;
extern volatile uint32_t debug_delta;
#endif


/*------------------------------------------------------------------------------
 Exported function prototypes                                             
------------------------------------------------------------------------------*/

void HAL_TIM_MspPostInit
	(
	TIM_HandleTypeDef *htim
	);


/*------------------------------------------------------------------------------
 Function prototypes                                             
------------------------------------------------------------------------------*/

/* telemetry.c */
void telemetry_build_payload
    (
    LORA_MESSAGE*       msg_buf,      /* o: buffer passed by caller        */
    uint32_t*           timestamp,    /* i: time since launch detect start */
    LORA_MESSAGE_TYPES  message_type  /* i: what kind of message           */
    );

#ifdef __cplusplus
}
#endif

#endif /* __TELEM_H */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/