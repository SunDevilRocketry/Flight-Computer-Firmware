/*******************************************************************************
*
* FILE: 
* 		terminal.h
*
* DESCRIPTION: 
* 	    Contains the pre-processing, execution, and post-processing of terminal
*       commands and data for the dual deploy firmware 
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TERMINAL_H 
#define TERMINAL_H 

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------------------------------------------
 Includes 
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Macros 
------------------------------------------------------------------------------*/

/* Dual deploy subcommand opcodes */
#define DUAL_DEPLOY_OP_STATUS    0x01 /* status subcommand  */
#define DUAL_DEPLOY_OP_EXTRACT   0x02 /* extract subcommand */


/*------------------------------------------------------------------------------
 Typdefs 
------------------------------------------------------------------------------*/

/* Return response codes */
typedef enum _TERMINAL_STATUS
    {
    TERMINAL_OK               , /* Terminal command successful        */
    TERMINAL_SENSOR_ERROR     , /* Terminal sensor command error      */
    TERMINAL_IGN_ERROR        , /* Terminal ignition command error    */
    TERMINAL_FLASH_ERROR      , /* Terminal flash command error       */
    TERMINAL_UNRECOGNIZED_CMD , /* Terminal invalid command           */
    TERMINAL_DUAL_DEPLOY_ERROR, /* Terminal dual deploy command error */
    TERMINAL_DATA_LOG_ERROR   , /* Data logger error                  */
    TERMINAL_ERROR        
    } TERMINAL_STATUS;


/*------------------------------------------------------------------------------
 Function Prototypes 
------------------------------------------------------------------------------*/

/* Executes a terminal command */
TERMINAL_STATUS terminal_exec_cmd
    (
    uint8_t command
    );

/* Executes a dual deploy command */
TERMINAL_STATUS dual_deploy_cmd_execute
    (
    uint8_t subcommand
    );

#ifdef __cplusplus
}
#endif

#endif /* TERMINAL_H */

/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/