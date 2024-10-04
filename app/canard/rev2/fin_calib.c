/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    fin_calib.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Calibrates left and right fin individually.                            *
*                                                                              *
*******************************************************************************/

#include <stdint.h>
#include "main.h"
#include "usb.h"

/*------------------------------------------------------------------------------
Define cases                                                                  
------------------------------------------------------------------------------*/
typedef enum _FIN_CALI_SUBCOM{
    LEFT_POS = 0x01,
    LEFT_NEG = 0x02,
    RIGHT_POS = 0x03,
    RIGHT_NEG = 0x04,
    EXIT = 0x05,
} FIN_CALI_SUBCOM;

/*------------------------------------------------------------------------------
Declaration                                                                  
------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
fin calib                                                                  
------------------------------------------------------------------------------*/
void finCalibration(FSM_STATE* pState) 
{

    while (*pState == FSM_FIN_CALIB_STATE) 
    {
        FIN_CALI_SUBCOM subcommand;
        USB_STATUS usb_status = usb_receive(&subcommand, sizeof(subcommand), HAL_DEFAULT_TIMEOUT);

        // if USB_STATUS == ERROR -> error handler

        switch(subcommand) 
        {
            case LEFT_NEG:       // Commented out for now. Please re-include when it'll make correctly.
                // servo.turn(-1);     // insert real function here
                break;
            case LEFT_POS:
                // servo.turn(1);
                break;
            case RIGHT_NEG:
                // servo.turn(-1);
                break;
            case RIGHT_POS:
                // servo.turn(1);
                break;
            case EXIT:
                *pState = FSM_IDLE_STATE;
            default:
                break;
        }
    }
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/