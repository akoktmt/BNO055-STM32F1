/*
 * LKF.h
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */

#ifndef LKF_H_
#define LKF_H_

#define LENGTH_REAR 10
#define LENGTH_CAR 20
#define SIZE 6
#define NUMBEROFMODLE 6
#define PI 3.141592
#include <math.h>
typedef struct{
	float FriPx;
	float FriPy;
	float FriVelx;
	float FriVely;
	float FriHea;
	float FriStee;

	float NexPx;
	float NexPy;
	float NexVelx;
	float NexVely;
	float NexHea;
	float NexStee;

	float CovPx;
	float CovPy;
	float CovVelx;
	float CovVely;
	float CovHea;
	float CovStee;

	float Prediction_CovarianceNex[NUMBEROFMODLE][NUMBEROFMODLE];
	float Prediction_CovarianceFri[NUMBEROFMODLE][NUMBEROFMODLE];
}LKF;

typedef struct {
	float Velx;
	float Vely;
	float Velz;
	float Stee;
	float Head;
	float Accx;
	float Accy;
	float Accz;
	float Time;
}Input;

typedef struct{
	float AngleBeta;
	float Roll;
	float Pitch;
	float Yaw;
}Angle;

typedef struct{
	float GPSGetPosition[2][2];
	float GPSCovariance[2][2];
	uint8_t GPS_Model[NUMBEROFMODLE][NUMBEROFMODLE];
}GPS;

typedef struct{
	float Yaw;
	float Covariane;
}Heading;

void transposeMatrixNxM(int row, int col, float mat[row][col], float trans[col][row]);
void transposeMatrixNxN(float mat[SIZE][SIZE], float result[SIZE][SIZE]);
void matrixMultiplication(float mat1[SIZE][SIZE], float mat2[SIZE][SIZE], float result[SIZE][SIZE]);
void EKF_Init(LKF *LKF,Input *Input);
void GPS_Init(GPS *GPS);
void EKF_PredictionStep(LKF *LKF, Angle *Angle, Input *Input);
void EFK_GPSHandleMeasurement(GPS *GPS, LKF *LKF );
void assignMatrix(float source[SIZE][SIZE], float destination[SIZE][SIZE], float rows, float cols);
void subtractMatrices(float mat1[SIZE][SIZE], float mat2[SIZE][SIZE], float result[SIZE][SIZE], int rows, int cols);
void addScalarToMatrix(int rows,int cols,float matrix[rows][cols], float scalar);


#endif /* LKF_H_ */


