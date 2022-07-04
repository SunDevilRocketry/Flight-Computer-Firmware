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
#include "ignition.h"


/*------------------------------------------------------------------------------
 Procedures 
------------------------------------------------------------------------------*/


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
uint8_t ign_cmd_execute
	(
    uint8_t ign_subcommand
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_STAT ign_status = 0; /* Status code returned by ignite API function */

/*------------------------------------------------------------------------------
 Call API function 
------------------------------------------------------------------------------*/
switch(ign_subcommand)
	{
    /* Light ematch */
	case IGN_FIRE_CODE:
		ign_status = ignite();
		break;

	/* Return continuity information */
	case IGN_CONT_CODE:
		ign_status = ign_get_cont_info();
		break;

    /* Unrecognized subcommand code: call error handler */
	default:
		Error_Handler();

    }

/* Return the response code */
return ign_status;

} /* ign_cmd_execute */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ignite                                                                 *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Asserts the ignition signal to ignite the engine ematch. Returns a     *
*       response code indicating if the ignition occured succesfully           *
*                                                                              *
*******************************************************************************/
IGN_STAT ignite
    (
	void
    )
{
/*------------------------------------------------------------------------------
 API function implementation
------------------------------------------------------------------------------*/

/* Check for e-match/switch continuity */
if (!ematch_cont())
	{
    /* No continuity across ematch and/or switch */
    return IGN_FAIL_E_MASK; 
    }

/* Check that power supply is not USB */

/* Assert ignition signal for 10 ms */
HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_PIN, GPIO_PIN_SET);
HAL_Delay(100);
HAL_GPIO_WritePin(FIRE_GPIO_PORT, FIRE_PIN, GPIO_PIN_RESET);

/* Check ematch continuity to check that ematch was lit */
if (!ematch_cont())
	{
    return IGN_SUCCESS;
    }
else /* Ignition unsuccessful */
	{
    return IGN_FAIL_MASK;
    }

} /* ignite */


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
IGN_STAT ign_get_cont_info
	(
    void
    )
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
IGN_STAT ign_status = 0; /* Status code to be returned */


/*------------------------------------------------------------------------------
 Call API functions 
------------------------------------------------------------------------------*/

/* Poll the ematch continuity pin */
if (ematch_cont())
	{
    ign_status |= IGN_E_CONT_MASK;
    }

/* Poll the solid propellant continuity pin */
if (solid_prop_cont())
	{
    ign_status |= IGN_SP_CONT_MASK;
    }

/* Poll the nozzle continuity pin */
if (nozzle_cont())
	{
    ign_status |= IGN_NOZ_CONT_MASK;
    }

/* Return the status code */
return ign_status;
}


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		solid_prop_cont                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the solid propellant wire   *
*       screw terminals                                                        *
*                                                                              *
*******************************************************************************/
bool solid_prop_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t solid_prop_cont_pinstate = HAL_GPIO_ReadPin(SP_CONT_GPIO_PORT, SP_CONT_PIN);

/* Return true if GPIO state is high*/
if (solid_prop_cont_pinstate == 0)
	{
    return true;
	}
else
	{
    return false;
    }

} /* solid_prop_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		nozzle_cont                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the nozzle wire screw       * 
*       terminals                                                              *
*                                                                              *
*******************************************************************************/
bool nozzle_cont
	(
	void
	)
{

/* Check MCU GPIO State */
uint8_t nozzle_cont_pinstate = HAL_GPIO_ReadPin(NOZ_CONT_GPIO_PORT, NOZ_CONT_PIN);

/* Return true if GPIO state is high*/
if (nozzle_cont_pinstate == 0)
	{
    return true;
	}
else
	{
    return false;
    }

} /* nozzle_cont */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
* 		ematch_cont                                                            *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Returns TRUE if there is continuity across the ematch and switch screw * 
*       terminals                                                              *
*                                                                              *
*******************************************************************************/
bool ematch_cont
	(
	void
	)
{
/* Check MCU GPIO State */
uint8_t ematch_cont_pinstate = HAL_GPIO_ReadPin(E_CONT_GPIO_PORT, E_CONT_PIN);

/* Return true if GPIO state is low */
if (ematch_cont_pinstate == 0)
	{
    return false;
	}
else
	{
    return true;
    }

} /* ematch_cont */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/
