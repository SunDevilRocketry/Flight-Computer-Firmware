#include <cstdint>
/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    fin_calib.c                                                        *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Calibrates left and right fin individually                             *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
Define cases                                                                  
------------------------------------------------------------------------------*/

#define LEFT_POS    0x02
#define LEFT_NEG    0x03
#define RIGHT_POS   0x04
#define RIGHT_NEG   0x05
#define DONE        0x01

/*------------------------------------------------------------------------------
Declaration                                                                  
------------------------------------------------------------------------------*/

uint8_t cmd;

/*------------------------------------------------------------------------------
fin calib                                                                  
------------------------------------------------------------------------------*/
void finCalibration() {

    while(1) {
        cmd = radio_recieve; // insert real function here

            switch(cmd) {
                case LEFT_NEG:
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
                case DONE:
                    return 0x0;
            }
    }
}
/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/