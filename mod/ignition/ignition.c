/*******************************************************************************
*
* FILE: 
* 		ignition.c
*
* DESCRIPTION: 
* 		Contains API function to the engine controller ignition system and 
*       contintuity readings
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdbool.h>


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include "sdr_pin_defines_A0002.h"
#include "ignition.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/

#if defined TERMINAL
/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_cmd_execute                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Executes an ignition subcommand based on user input from the sdec      *
*       terminal                                                               *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_cmd_execute
	(
    IGN_SUBCOMMAND ign_subcommand
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_STATUS ign_status = 0; /* Status code returned by ignite API function */

/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/
switch( ign_subcommand )
	{
    /* Deploy main */
	case IGN_MAIN_DEPLOY_CODE:
		{
		ign_status = ign_deploy_main();
		break;
		}

    /* Deploy drogue */
	case IGN_DROGUE_DEPLOY_CODE:
		{
		ign_status = ign_deploy_main();
		break;
		}

	/* Return continuity information */
	case IGN_CONT_CODE:
		{
		ign_status = ign_get_cont_info();
		break;
		}

    /* Unrecognized subcommand code: call error handler */
	default:
		{
		Error_Handler();
		break;
		}
    }

/* Return the response code */
return ign_status;

} /* ign_cmd_execute */

#endif /* TERMINAL */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_deploy_main                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the main parachute deployment    *
*       ematch. Returns a response code indicating if the ignition occured     *
*       succesfully                                                            *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_deploy_main 
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check continuities before deploying*/
if      ( !ign_switch_cont() )
	{
	return IGN_SWITCH_FAIL;
	}
else if ( !ign_main_cont()   )
	{
    return IGN_MAIN_CONT_FAIL; 
    }
else /* Continuity is good for main */
	{
	/* Assert ignition signal for 10 ms */
	HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_SET );
	HAL_Delay( IGN_BURN_DELAY );
	HAL_GPIO_WritePin( MAIN_GPIO_PORT, MAIN_PIN, GPIO_PIN_RESET );
	}

/* Check ematch continuity to check that ematch was lit */
if ( !ign_main_cont() )
	{
	return IGN_OK;
	}
else /* Ignition unsuccessful */
	{
	return IGN_MAIN_FAIL;
	}

} /* ign_deploy_main */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_deploy_drogue                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the drogue parachute deployment  *
*       ematch. Returns a response code indicating if the ignition occured     *
*       succesfully                                                            *
*                                                                              *
*******************************************************************************/
IGN_STATUS ign_deploy_drogue 
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check continuities before deploying*/
if      ( !ign_switch_cont() )
	{
	return IGN_SWITCH_FAIL;
	}
else if ( !ign_drogue_cont()   )
	{
    return IGN_DROGUE_CONT_FAIL; 
    }
else /* Continuity is good for drogue */
	{
	/* Assert ignition signal for 10 ms */
	HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_SET   );
	HAL_Delay( 10 );
	HAL_GPIO_WritePin( DROGUE_GPIO_PORT, DROGUE_PIN, GPIO_PIN_RESET );
	}

/* Check ematch continuity to check that ematch was lit */
if ( !ign_drogue_cont() )
	{
	return IGN_OK;
	}
else /* Ignition unsuccessful */
	{
	return IGN_DROGUE_FAIL;
	}

} /* ign_deploy_drogue */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_get_cont_info                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Polls each continuity pin and sets the continuity bits in the          *
*       response code                                                          *   
*                                                                              *
*******************************************************************************/
IGN_CONT_STAT ign_get_cont_info
	(
    void
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_CONT_STAT ign_status = 0; /* Status code to be returned */


/*------------------------------------------------------------------------------
 Call API functions 
------------------------------------------------------------------------------*/

/* Poll the switch continuity pin */
if ( ign_switch_cont() )
	{
    ign_status |= IGN_SWITCH_MASK;
    }

/* Poll the main parachute deployment continuity pin */
if ( ign_main_cont() )
	{
    ign_status |= IGN_MAIN_CONT_MASK;
    }

/* Poll the drogue parachute deployment continuity pin */
if ( ign_drogue_cont() )
	{
    ign_status |= IGN_DROGUE_CONT_MASK;
    }

/* Return the status code */
return ign_status;

} /* ign_get_cont_info */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_main_cont                                                          *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the main parachute          *
*       deployment ematch                                                      *
*                                                                              *
*******************************************************************************/
bool ign_main_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t main_cont_pinstate = HAL_GPIO_ReadPin( MAIN_GPIO_PORT, MAIN_PIN );

/* Return true if GPIO state is high*/
if ( main_cont_pinstate == 0 )
	{
    return true;
	}
else
	{
    return false;
    }

} /* ign_main_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_drogue_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the drogue parachute        * 
*       deployment ematch                                                      *
*                                                                              *
*******************************************************************************/
bool ign_drogue_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t drogue_cont_pinstate = HAL_GPIO_ReadPin( DROGUE_CONT_GPIO_PORT, 
                                                 DROGUE_CONT_PIN );

/* Return true if GPIO state is high*/
if ( drogue_cont_pinstate == 0 )
	{
    return true;
	}
else
	{
    return false;
    }

} /* drogue_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ign_switch_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the switch screw terminals  * 
*                                                                              *
*******************************************************************************/
bool ign_switch_cont
	(
	void
	)
{
/* Check MCU GPIO State */
uint8_t switch_cont_pinstate = HAL_GPIO_ReadPin(SWITCH_GPIO_PORT, SWITCH_PIN);

/* Return true if GPIO state is low */
if ( switch_cont_pinstate == 0 )
	{
    return false;
	}
else
	{
    return true;
    }

} /* switch_cont */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
