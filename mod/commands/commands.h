/*******************************************************************************
*
* FILE: 
* 		commands.c
*
* DESCRIPTION: 
* 		Contains general command functions common to all embedded controllers
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMANDS_H
#define COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* sdec command codes */
#define PING_OP 	0x01    /* ping command opcode     */
#define CONNECT_OP	0x02    /* connect command opcode  */
#define IGNITE_OP	0x20    /* ignition command opcode */
#define FLASH_OP    0x22    /* flash command opcode    */
#define SENSOR_OP   0x03    /* sensor command opcode   */

/* Board identifier code */
#ifdef A0002_REV1
	/* Rev 1 */
	#define A0002_PING_RESPONSE_CODE    ( 0x04 )
#endif


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Sends a single response byte back to sender */
void ping
	(
	void
	);


#endif /* COMMANDS_H */
