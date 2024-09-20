/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    fsm_canard.h                                                       *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Canard definitions                                                     *
*                                                                              *
*******************************************************************************/
/* State Extern */
#ifndef FSM_STATE
typedef enum
	{
	FSM_IDLE_STATE       	  , 
    FSM_FIN_CALIB_STATE       ,
    FSM_IMU_CALIB_STATE       ,
	FSM_PID_CONTROL_STATE     ,
    FSM_ABORT_STATE
	} FSM_STATE;

extern FSM_STATE canard_controller_state;
#endif

/* Signals */

#ifndef FSM_IMU_CALIB_TRIGGER
#define FSM_IMU_CALIB_TRIGGER        (0x00000001)
#endif
#ifndef FSM_FIN_CALIB_TRIGGER
#define FSM_FIN_CALIB_TRIGGER        (0x00000002)
#endif
#ifndef FSM_PID_CONTROL_TRIGGER
#define FSM_PID_CONTROL_TRIGGER      (0x00000003)
#endif
#ifndef FSM_IDLE_RETURN_TRIGGER
#define FSM_IDLE_RETURN_TRIGGER      (0x00000004)
#endif

/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/