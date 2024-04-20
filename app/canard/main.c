/*******************************************************************************
*
* FILE: 
* 		main.c
*
* DESCRIPTION: 
* 		Fin Calibration and PID - Implements basic Fin Calibration and PID control 
*                     
*
*******************************************************************************/



void main()
{
    //get the state from the controller
    //run a while loop
    //in the while loop set up cases based on the states from the controller
    //that then runs the different methods for fin_calibration, imu_calibration, pid_control

 

    // finCalibration();
    imuCalibration();
    pid();
}