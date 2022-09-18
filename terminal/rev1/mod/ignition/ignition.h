/*******************************************************************************
*
* FILE: 
* 		ignition.h
*
* DESCRIPTION: 
* 		Contains API function to the engine controller ignition system and 
*       contintuity readings
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

/* Ignition response code */
/* IGN_STAT = bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 

   bit7: not used
   bit6: Ignition status, success (1) or fail (0)
   bit5: Ignition failure due to ematch continuity detected after ignition 
         signal is asserted, failure (1) or pass (0)
   bit4: Ignition failure due to missing power supply, failure (1) or pass (0)
   bit3: Ignition failure due to ematch discontinuity, failure (1) or pass (0)
   bit2: Nozzle wire continuity, 1 indicates continuity between screw terminals
   bit1: Solid propellant wire continuity, 1 indicates continuity between screw 
         terminals
   bit0: Ematch/switch continuity
                                                                     */
typedef uint8_t IGN_STAT;



/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Ignition subcommand codes */
#define IGN_FIRE_CODE	    0x01
#define IGN_CONT_CODE	    0x02

/* Ignition response code bitmasks */
#define IGN_E_CONT_MASK   	0b00000001
#define IGN_SP_CONT_MASK  	0b00000010
#define IGN_NOZ_CONT_MASK 	0b00000100
#define IGN_FAIL_E_MASK   	0b00001000
#define IGN_FAIL_PWR_MASK 	0b00010000
#define IGN_FAIL_MASK       0b00100000
#define IGN_SUCCESS         0b01000000


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/


/* Ignite Engine to initialize combustion                            */
IGN_STAT ignite
	(
    void
	); 

/* Poll continuity terminals and report continuity info              */
IGN_STAT ign_get_cont_info
	(
    void
    );

/* Check for continuity across solid propellant wire screw terminals */
bool solid_prop_cont
	(
    void
    );

/* Check for continuity across nozzle wire screw terminals           */
bool nozzle_cont
	(
    void
    );

/* Check for continuity across ematch and switch screw terminals     */
bool ematch_cont
	(
    void
    );

/* Execute a terminal command using API functions */
uint8_t ign_cmd_execute
	(
    uint8_t ign_subcommand
    );


#endif /* IGNITION_H */
