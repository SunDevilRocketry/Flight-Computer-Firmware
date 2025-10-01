/*******************************************************************************
*                                                                              *
* FILE:                                                                        * 
* 		    gyro_kalman.c                                                      *
*                                                                              *
* DESCRIPTION:                                                                 * 
* 		Data smoothing and state estimation for gyro data                      *
*                                                                              *
*******************************************************************************/

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/
extern SENSOR_DATA 	sensor_data;
extern uint8_t 	   	acc_detect_flag;

/*------------------------------------------------------------------------------
Structs (temporary till i found out how eli likes his code)                                                                  
------------------------------------------------------------------------------*/
typedef struct _GYRO_STATE
    {
    int16_t phi_rad;
    int16_t theta_rad;
    int16_t psi_rad;

    int16_t P[9];
    int16_t Q[3];
    int16_t R[3];
    } GYRO_STATE;

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		gyro_kalman_filter                                                       *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		Gyro smoothing/prediction of roll, yaw, pitch                            * 
*       Predicts gyro data, then corrects based off measurement, repeat          *
*                                                                                *
* NOTE:                                                                          *
*       Contains a lot of math :(                                                *
*                                                                                *
*********************************************************************************/

void matmul3_3 (int16_t A[9], int16_t B[9], int16_t C[9]){
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                C[i * 3 + j] = A[i * 3 + k] * B[k * 3 + j];
            }
        }
    }

}

void transpose3_3 (int16_t A[9], int16_t B[9]){
    for (int i = 0; i<3; i++){
        for (int j = 0; j < 3; j++){
            B[j * 3 + i] = A[i * 3 + j];
        }
    }
}

void scalarmul3_3 (int16_t A, int16_t B[9], int16_t C[9]){
    for (int i = 0; i<3; i++){
        for (int j = 0; j < 3; j++){
            C[i * 3 + j] = B[i * 3 + j]*A;
        }
    }
}

void matadd3_3 (int16_t A[9], int16_t B[9], int16_t C[9]){
    for (int i = 0; i<3; i++){
        for (int j = 0; j < 3; j++){
            C[i * 3 + j] = B[i * 3 + j] + A[i * 3 + j];
        }
    }
}

void gyro_kalman_filter_Init (GYRO_STATE *state, int16_t Pinit, int16_t *Qinit, int16_t *Rinit){

    //angles of displacement (read euler angles)
    state -> phi_rad = 0;
    state -> theta_rad = 0;
    state -> psi_rad = 0;

    //Covariance matrix, initialized as diagonal
    state -> P[0] = Pinit; state -> P[1] = 0;     state -> P[2] = 0;
    state -> P[3] = 0;     state -> P[4] = Pinit; state -> P[5] = 0;
    state -> P[6] = 0;     state -> P[7] = 0;     state -> P[8] = Pinit;

    state -> Q[0] = Qinit[0]; state -> Q[1] = Qinit[1]; state -> Q[2] = Qinit[2];

    state -> R[0] = Rinit[0]; state -> R[1] = Rinit[1]; state -> R[2] = Rinit[2];
}

//0 is y, 1 is z, 2 is x
void gyro_kalman_filter_predict (GYRO_STATE *state, int16_t T){

    //phi
    int16_t p = sensor_data.imu_data.imu_converted.gyro_y;
    //theta
    int16_t q = sensor_data.imu_data.imu_converted.gyro_z;
    //psi
    int16_t r = sensor_data.imu_data.imu_converted.gyro_x;

    //Update trig values 
    int16_t sinphi = (int16_t)sin((double)state->phi_rad);
    int16_t cosphi = (int16_t)cos((double)state->phi_rad);

    int16_t tantheta = (int16_t)tan((double)state->phi_rad);
    int16_t sectheta = 1/(int16_t)cos((double)state->phi_rad);

    //This is taken from Euler Angles, basically convert change of angle into physical rotational displacement
    //Also called the update function
    state->phi_rad = state->phi_rad + T * (p + sinphi*tantheta*q + cospsi*tantheta*r);
    state->theta_rad = state->theta_rad + T * (cosphi*q - sinphi*r);
    state->psi_rad = state->psi_rad + T * (sinphi*sectheta*q + cosphi*sectheta*r);

    //reupdate trig values with new Euler Angles
    int16_t sinphi = (int16_t)sin((double)state->phi_rad);
    int16_t cosphi = (int16_t)cos((double)state->phi_rad);

    int16_t costheta = (int16_t)cos((double)state->phi_rad);
    int16_t tantheta = (int16_t)tan((double)state->phi_rad);
    int16_t sectheta = 1/(int16_t)cos((double)state->phi_rad);
    
    int16_t sinpsi = (int16_t)sin((double)state->psi_rad);
    int16_t cospsi = (int16_t)cos((double)state->psi_rad);

    //Jacobian of the update function, I would rather die than make a function to calculate this in C
    int16_t A[9] = 
    {
        q*cosphi*tantheta - r*sinphi*tantheta, q*(1/(costheta*costheta))*sinphi + r*(1/(costheta*costheta))*cosphi, 0,
        -1*q*sinphi - r*cosphi,                0,                                                                    0,
        0,                                     q*sinpsi*tantheta*(1/costheta) + r*cospsi*tantheta*(1/costheta),      q*cospsi*sectheta - r*sinpsi*(1/costheta)
    }

    //A * P
    int16_t AP[9];
    matmul3_3(A, state->P, AP);

    //A' = A transpose
    int16_t Atrans[9];
    transpose3_3(A, Atrans);

    //P*A'
    int16_t PAtrans[9];
    matmul3_3(state->P, Atrans, PAtrans);

    //AP + PA'
    int16_t APPAt[9];
    matadd3_3(AP, PAtrans, APPAt);

    //AP + PA' + Q
    int16_t APPAtQ[9];
    matadd3_3(APPAt, state->Q, APPAtQ);

    //T * (AP + PA' + Q)
    int16_t Ptemp[9];
    scalarmul3_3(state->T, APPAtQ, Ptemp);


    //P+ = P- + T * (A*P- + P-*A' + Q)
    //Update Covariance Matrix
    matadd3_3(state->P, Ptemp, state->P);
}

void gyro_kalman_filter_update (GYRO_STATE *state) {
    
}