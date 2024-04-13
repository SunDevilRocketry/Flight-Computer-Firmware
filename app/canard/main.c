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
/*
*  Runs methods from fin_calib.c, imu_calib.c, pid_control.c 
*/
void main()
{
    finCalibration();
    imuCalibration();
    pid();
}