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
#include <string.h>

/*------------------------------------------------------------------------------
Instantiations                                                                  
------------------------------------------------------------------------------*/
extern SENSOR_DATA 	sensor_data;

/*------------------------------------------------------------------------------
Structs (temporary till i found out how eli likes his code)                                                                  
------------------------------------------------------------------------------*/
typedef struct _GYRO_STATE
    {
    //Euler rate definitions
    float phi_rad;
    float theta_rad;
    float psi_rad;

    //Error covariance matrix
    float P[9];
    //Model noise (roll)
    float Q[3];
    //Model noise (accel)
    float R[3];
    //Sample time
    float T;
    } GYRO_STATE;

/*********************************************************************************
*                                                                                *
* FUNCTION:                                                                      * 
* 		linalg functions                                                         *
*                                                                                *
* DESCRIPTION:                                                                   * 
* 		various matrix and vector operations                                     * 
*       Used heavily in kalman filter                                            *
*                                                                                *
* NOTE:                                                                          *
*       Contains a lot of math :(                                                *
*                                                                                *
*********************************************************************************/

//copies A into B
//read as "this copies A into B"
void matcopy (float A[9], float B[9]){
    memcpy(B, A, 9*sizeof(double));
}

//Given matricies A and B, set C to A*B
void matmul3_3 (float A[9], float B[9], float C[9]){
    float ibtw[9];
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            for (int k = 0; k < 3; k++){
                ibtw[i * 3 + j] = A[i * 3 + k] * B[k * 3 + j];
            }
        }
    }
    matcopy(ibtw, C);
}

//Given matrix A, make B its transpose
void transpose3_3 (float A[9], float B[9]){
    float ibtw[9];
    for (int i = 0; i<3; i++){
        for (int j = 0; j < 3; j++){
            ibtw[j * 3 + i] = A[i * 3 + j];
        }
    }
    matcopy(ibtw, B);
}

//Given scalar value A and matrix B, set C to A*B
void scalarmul3_3 (float A, float B[9], float C[9]){
    float ibtw[9];
    for (int i = 0; i<3; i++){
        for (int j = 0; j < 3; j++){
            ibtw[i * 3 + j] = B[i * 3 + j]*A;
        }
    }
    matcopy(ibtw, C);
}

//given matricies A and B, set C to A+B
void matadd3_3 (float A[9], float B[9], float C[9]){
    float ibtw[9];
    for (int i = 0; i<3; i++){
        for (int j = 0; j < 3; j++){
            ibtw[i * 3 + j] = B[i * 3 + j] + A[i * 3 + j];
        }
    }
    matcopy(ibtw, C);
}

//Given matrix A and diagonal matrix B, set C to A+B
//We need this because I defined my diagonal matricies as vectors :( )
void matadd_vect (float A[9], float B[3], float C[9]){
    float ibtw[9];
    ibtw[0] = A[0] + B[0];
    ibtw[4] = A[4] + B[1];
    ibtw[8] = A[8] + B[2];
    matcopy(ibtw, C);
}

//Given matrix A and vector B, set C to A*B
void matmul_vect (float A[9], float B[3], float C[9]){
    float ibtw[9];
    for (int i = 0; i < 3; i++){
        ibtw[i] = 0;
        for (int j = 0; j < 3; j++){
            ibtw[i] += A[i * 3 + j] * B[j];
        }
    }
    matcopy(ibtw, C);
}

//invert a matrix
//Given matrix A, set B as the inverse
int matinv (float A[9], float B[9]){
    double det = A[0]*(A[4]*A[8] - A[5]*A[7]) - A[1]*(A[3]*A[8] - A[5]*A[6]) + A[2]*(A[3]*A[7] - A[4]*A[6]);

    if (det == 0.0) {
        return 0;
    }

    double invdet = 1.0 / det;

    double ibtw[9];
    ibtw[0] =       A[4]*A[8] - A[5]*A[7];
    ibtw[1] = -1.0*(A[1]*A[8] - A[2]*A[7]);
    ibtw[2] =       A[1]*A[5] - A[2]*A[4];
    ibtw[3] = -1.0*(A[3]*A[8] - A[5]*A[6]);
    ibtw[4] =       A[0]*A[8] - A[2]*A[6];
    ibtw[5] = -1.0*(A[0]*A[5] - A[2]*A[3]);
    ibtw[6] =       A[3]*A[7] - A[4]*A[6];
    ibtw[7] = -1.0*(A[0]*A[7] - A[1]*A[6]);
    ibtw[8] =       A[0]*A[4] - A[1]*A[3];

    scalarmul3_3(invdet, ibtw, B);
    return 1;
}

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
*       Contains a lot of linear algebra :(                                      *
*                                                                                *
*********************************************************************************/
void gyro_kalman_filter_Init (GYRO_STATE *state, float Pinit, float *Qinit, float *Rinit){

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
void gyro_kalman_filter_predict (GYRO_STATE *state, float T){

    //phi
    float p = sensor_data.imu_data.imu_converted.gyro_y;
    //theta
    float q = sensor_data.imu_data.imu_converted.gyro_z;
    //psi
    float r = sensor_data.imu_data.imu_converted.gyro_x;

    //Update trig values 
    float sinphi = sin(state->phi_rad);
    float cosphi = cos(state->phi_rad);

    float tantheta = tan(state->phi_rad);
    float sectheta = 1/cos(state->phi_rad);

    float cospsi = cos(state->psi_rad);

    //This is taken from Euler Angles, basically convert change of angle into physical rotational displacement
    //Also called the update function
    state->phi_rad = state->phi_rad + T * (p + sinphi*tantheta*q + cospsi*tantheta*r);
    state->theta_rad = state->theta_rad + T * (cosphi*q - sinphi*r);
    state->psi_rad = state->psi_rad + T * (sinphi*sectheta*q + cosphi*sectheta*r);

    //reupdate trig values with new Euler Angles
    float sinphi = sin(state->phi_rad);
    float cosphi = cos(state->phi_rad);

    float costheta = cos(state->phi_rad);
    float tantheta = tan(state->phi_rad);
    float sectheta = 1/cos(state->phi_rad);
    
    float sinpsi = sin(state->psi_rad);
    float cospsi = cos(state->psi_rad);

    //Jacobian of the update function, I would rather die than make a function to calculate this in C
    float A[9] = 
    {
        q*cosphi*tantheta - r*sinphi*tantheta, q*(1/(costheta*costheta))*sinphi + r*(1/(costheta*costheta))*cosphi,  0,
        -1*q*sinphi - r*cosphi,                0,                                                                    0,
        0,                                     q*sinpsi*tantheta*(1/costheta) + r*cospsi*tantheta*(1/costheta),      q*cospsi*sectheta - r*sinpsi*(1/costheta)
    };

    //A * P
    float AP[9];
    matmul3_3(A, state->P, &AP);

    //A' = A transpose
    float Atrans[9];
    transpose3_3(A, &Atrans);

    //P*A'
    float PAtrans[9];
    matmul3_3(state->P, Atrans, &PAtrans);

    //AP + PA'
    float APPAt[9];
    matadd3_3(AP, PAtrans, &APPAt);

    //AP + PA' + Q
    float APPAtQ[9];
    matadd_vect(APPAt, state->Q, &APPAtQ);

    //T * (AP + PA' + Q)
    float Ptemp[9];
    scalarmul3_3(state->T, APPAtQ, &Ptemp);


    //P+ = P- + T * (A*P- + P-*A' + Q)
    //Update Covariance Matrix
    matadd3_3(state->P, Ptemp, &state->P);
}

void gyro_kalman_filter_update (GYRO_STATE *state) {
    
    //x acceleration
    float ax = sensor_data.imu_data.imu_converted.accel_y;
    //y acceleration
    float ay = sensor_data.imu_data.imu_converted.accel_z;
    //z acceleration
    float az = sensor_data.imu_data.imu_converted.accel_x;
    //gravity constant
    float g = 9.81;

    //common trig functions
    float sinphi = sin(state->phi_rad);
    float cosphi = cos(state->phi_rad);

    float sintheta = sin(state->theta_rad);
    float costheta = cos(state->phi_rad);    

    //output function H. basically what the kalman model of our sensors predict
    float H[3] = (
        g*sintheta,
        -1*g*sinphi*costheta,
        -1*g*cosphi*costheta
    );

    //the next step depends on h(x, u), the output function. We take the jacobian (partial derivative of elements of H with respect to elements of X)
    float C[9] = {
                    0,                      g*costheta,          0,
                    -1*g*cosphi*costheta,   g*sinphi*sintheta,   0,
                    g*sinphi*costheta,      g*sintheta*cosphi,   0
                };

    //Kalman Gain K = P * C' * (C * P * C' + R)^-1 (-1 denotes inverse, ' denotes transpose)
    //Transpose of C
    float Ct[9];
    transpose3_3(C, &Ct);
    //P * C'
    float PCt[9];
    matmul3_3(state->P,Ct, &PCt);
    //Parenthesis part
    float Pths[9];
    //Multiply for C * P * C'
    matmul3_3(C, PCt, Pths);
    //Add R
    matadd_vect(Pths, state->R, Pths);
    //Invert
    matinv(Pths, Pths);
    //Find K
    float K[9];
    matmul3_3(PCt, Pths, K);

    //Update the Covariance P++ = (I - K * C) * P+
    float Ptemp[9];
    float I[9] = (-1,0,0,0,-1,0,0,0,-1);
    //Back to parenthesis
    float Pths[9];
    matadd3_3(I, K, Pths);
    matmul3_3(Pths, C, Pths);
    //mul with P
    matmul3_3(Pths, state->P, Ptemp);
    //Add with normal P
    matadd3_3(state->P, Ptemp, state->P);

    //update state estimate x++ = x+ + K(y-h)
    //y is just the actual sensor output (so y-h is error between the two)
    //non functioned scalar subtraction because im lazy
    float Pths[3] = (
        ax - H[0],
        ay - H[1],
        az - H[2]
    );
    float xtemp[3];
    //K * (y-h)
    matmul_vect(K, Pths, xtemp);
    
    //update state estimate values
    state->phi_rad = state->phi_rad + xtemp[0];
    state->theta_rad = state->theta_rad + xtemp[1];
    state->psi_rad = state->psi_rad + xtemp[2];
    

}