/*******************************************************************************
*
* FILE: 
* 		ignition.h
*
* DESCRIPTION: 
* 		Contains API functions to the flight computer parachute deployment 
*       system and contintuity readings
*
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef IGNITION_H
#define IGNITION_H

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Ignition Continuity Status */
/* IGN_CONT_STAT = bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 

   bit3-7: not used 
   bit2:   Drogue parachute deployment continuity, 1 indicates continuity between 
           screw terminals
   bit1:   Main Parachute Deployment continuity, 1 indicates continuity between 
		   screw terminals
   bit0:   Switch continuity */
typedef uint8_t IGN_CONT_STAT;

/* Ignition Status Response Code */
typedef enum IGN_STATUS
	{
	IGN_OK = 0x40       ,
	IGN_FAIL = 0x20     ,
	IGN_SWITCH_FAIL     ,
	IGN_MAIN_FAIL       ,
	IGN_DROGUE_FAIL     ,
	IGN_MAIN_CONT_FAIL = 0x08,
    IGN_DROGUE_CONT_FAIL
	} IGN_STATUS;

/* SDEC Subcommand Codes */
typedef enum IGN_SUBCOMMAND
	{
	IGN_MAIN_DEPLOY_CODE = 0x01,
	IGN_CONT_CODE              ,
	IGN_DROGUE_DEPLOY_CODE     
	} IGN_SUBCOMMAND;


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Ignition response code bitmasks */
#define IGN_SWITCH_MASK   	    0b00000001
#define IGN_MAIN_CONT_MASK  	0b00000010
#define IGN_DROGUE_CONT_MASK 	0b00000100

/* Ignition burn time */
#define IGN_BURN_DELAY          10 


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

#if defined TERMINAL
/* Executes an ignition subcommand based on user input from the sdec terminal */
IGN_STATUS ign_cmd_execute
	(
    IGN_SUBCOMMAND ign_subcommand
    );
#endif


/* Asserts the ignition signal to ignite the main parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_main 
    (
	void
    );


/* Asserts the ignition signal to ignite the drogue parachute deployment ematch. 
   Returns a response code indicating if the ignition occured succesfully */
IGN_STATUS ign_deploy_drogue 
    (
	void
    );


/* Polls each continuity pin and sets the continuity bits in the response 
   code */
IGN_CONT_STAT ign_get_cont_info
	(
    void
    );


/* Returns TRUE if there is continuity across the main parachute deployment 
   ematch */
bool ign_main_cont
	(
	void
	);


/* Returns TRUE if there is continuity across the drogue parachute deployment 
   ematch */
bool ign_drogue_cont
	(
	void
	);


/* Returns TRUE if there is continuity across the switch screw terminals */
bool ign_switch_cont
	(
	void
	);

#endif /* IGNITION_H */

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
