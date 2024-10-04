#include <stdint.h>
#include "main.h"
/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    fin_calib.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Calibrates left and right fin individually.                            *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
Define cases                                                                  
------------------------------------------------------------------------------*/

#define LEFT_POS    0x12 /* Temp values */
#define LEFT_NEG    0x13
#define RIGHT_POS   0x14
#define RIGHT_NEG   0x15

/*------------------------------------------------------------------------------
Declaration                                                                  
------------------------------------------------------------------------------*/

uint8_t cmd;

/*------------------------------------------------------------------------------
fin calib                                                                  
------------------------------------------------------------------------------*/
void finCalibration(FSM_STATE* pState) 
{

    while(*pState = FSM_FIN_CALIB_STATE) 
    {
        uint8_t buffer;
        usb_receive(&buffer);

        switch(buffer) 
        {
            /* case LEFT_NEG:       // Commented out for now. Please re-include when it'll make correctly.
                servo.turn(-1);     // insert real function here
                break;
            case LEFT_POS:
                servo.turn(1);
                break;
            case RIGHT_NEG:
                servo.turn(-1);
                break;
            case RIGHT_POS:
                servo.turn(1);
                break;
            */
            case FSM_IDLE_RETURN_OPCODE:
                *pState = FSM_IDLE_STATE;
                return;
        }
    }
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/