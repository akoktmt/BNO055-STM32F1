/*
 * LKF.c
 *
 *  Created on: Nov 11, 2023
 *      Author: win 10
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "LKF.h"

void assignMatrix(float source[SIZE][SIZE], float destination[SIZE][SIZE], float rows, float cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            destination[i][j] = source[i][j];
        }
    }
}

void subtractMatrices(float mat1[SIZE][SIZE], float mat2[SIZE][SIZE], float result[SIZE][SIZE], int rows, int cols) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            result[i][j] = mat1[i][j] - mat2[i][j];
        }
    }
}

void multiplyMatrixByScalar(float matrix[SIZE][SIZE], int rows, int cols, int scalar) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            matrix[i][j] *= scalar;
        }
    }
}

void swapRows(float mat[SIZE][SIZE], int row1, int row2) {
    for (int i = 0; i < SIZE; i++) {
        float temp = mat[row1][i];
        mat[row1][i] = mat[row2][i];
        mat[row2][i] = temp;
    }
}

// Hàm để chia một dòng của ma trận cho một số
void scaleRow(float mat[SIZE][SIZE], int row, float scalar) {
    for (int i = 0; i < SIZE; i++) {
        mat[row][i] /= scalar;
    }
}

// Hàm để thực hiện phép trừ một dòng nhân với một số nhân cho một dòng khác
void rowOperation(float mat[SIZE][SIZE], int row1, int row2, float scalar) {
    for (int i = 0; i < SIZE; i++) {
        mat[row1][i] -= scalar * mat[row2][i];
    }
}

// Hàm để tính ma trận nghịch đảo sử dụng phương pháp Gauss-Jordan
void inverseMatrix(float mat[SIZE][SIZE], float result[SIZE][SIZE]) {
    // Khởi tạo ma trận đơn vị
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            result[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    // Áp dụng phương pháp Gauss-Jordan
    for (int i = 0; i < SIZE; i++) {
        // Chia hàng i cho phần tử mat[i][i]
        float scalar = mat[i][i];
        scaleRow(mat, i, scalar);
        scaleRow(result, i, scalar);

        // Loại bỏ các phần tử khác 0 tại cột i
        for (int j = 0; j < SIZE; j++) {
            if (i != j) {
                scalar = mat[j][i];
                rowOperation(mat, j, i, scalar);
                rowOperation(result, j, i, scalar);
            }
        }
    }
}

void addScalarToMatrix(int rows,int cols,float matrix[rows][cols], float scalar) {
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            matrix[i][j] += scalar;
        }
    }
}

void transposeMatrixNxM(int row, int col, float mat[row][col], float trans[col][row]) {

  for (int i = 0; i < row; i++) {
    for (int j = 0; j < col; j++) {
      trans[j][i] = mat[i][j];
    }
  }

}

void transposeMatrixNxN(float mat[SIZE][SIZE], float result[SIZE][SIZE]) {
    for (uint8_t i = 0; i < SIZE; i++) {
        for (uint8_t j = 0; j < SIZE; j++) {
            result[j][i] = mat[i][j];
        }
    }
}

void matrixMultiplication(float mat1[SIZE][SIZE], float mat2[SIZE][SIZE], float result[SIZE][SIZE]) {
    for (uint8_t i = 0; i < SIZE; i++) {
        for (uint8_t j = 0; j < SIZE; j++) {
            result[i][j] = 0;
            for (uint8_t k = 0; k < SIZE; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void EKF_Init(LKF *LKF,Input *Input)
{
//First Step
	LKF->FriPx= 0;  //input
	LKF->FriPy=0; //input
	LKF->FriVelx=5; //input
 	LKF->CovVely=0; //input
	LKF->FriHea=-PI/6; //input
	LKF->FriStee=PI/6; //input
//Next Step
	LKF->NexPx=0;
	LKF->NexPy=0;
	LKF->NexVelx=0;
	LKF->NexVely=0;
	LKF->NexHea=0;
	LKF->NexStee=0;
//Covariance
	LKF->CovPx=0; //input
	LKF->CovPy=0; //input
	LKF->CovVelx=0;//input
	LKF->CovVely=0;//input
	LKF->CovHea=0; //input
	LKF->CovStee=0; //input
//FirstIput
	Input->Accx=0.2;
	Input->Accy=0.2;
	Input->Accz=0.2;
	Input->Time=0.1; //input
//Covariance
	memset(LKF->Prediction_CovarianceNex,0,sizeof(LKF->Prediction_CovarianceNex));
	memset(LKF->Prediction_CovarianceFri,0,sizeof(LKF->Prediction_CovarianceFri));
	LKF->Prediction_CovarianceFri[0][0]= LKF->CovPx;
	LKF->Prediction_CovarianceFri[1][1]= LKF->CovPy;
	LKF->Prediction_CovarianceFri[2][2]= LKF->CovVelx;
	LKF->Prediction_CovarianceFri[3][3]= LKF->CovVely;
	LKF->Prediction_CovarianceFri[4][4]= LKF->CovHea;
	LKF->Prediction_CovarianceFri[5][5]= LKF->CovStee;
}

void GPS_Init(GPS *GPS){

	memset(GPS->GPSCovariance,0,sizeof(GPS->GPSCovariance));
	memset(GPS->GPSGetPosition,0,sizeof(GPS->GPSGetPosition));
	memset(GPS->GPS_Model,0,sizeof(GPS->GPS_Model));

	GPS->GPSCovariance[0][0]=0; //input fromsensor
	GPS->GPSCovariance[1][1]=0; //input fromsensor

	GPS->GPSGetPosition[0][0]=106.802343; //input fromsensor

	GPS->GPSGetPosition[1][1]=10.869826;//input fromsensor

	GPS->GPS_Model[0][0]=1;
	GPS->GPS_Model[1][1]=1;

}
void EKF_PredictionStep(LKF *LKF, Angle *Angle, Input *Input){
// Prediction State
	float a_hx;
	float a_hy;

	 a_hx = Input->Accx * cos(Angle->Pitch) + Input->Accy * sin(Angle->Roll) * sin(Angle->Pitch) + Input->Accz * cos(Angle->Roll) * sin(Angle->Pitch);
	 a_hy = Input->Accy * cos(Angle->Roll) - Input->Accz * sin(Angle->Roll);

	 Angle->AngleBeta= (atan(LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR);

	 LKF->NexPx= LKF->FriPx + (LKF->FriVelx * cos(Angle->AngleBeta+LKF->FriHea))*Input->Time;

	 LKF->NexPy= LKF->FriPy + (LKF->FriVely * sin(Angle->AngleBeta +LKF->FriHea))*Input->Time;

	 LKF->NexVelx= LKF->FriVelx + a_hx *Input->Time;

	 LKF->NexVely= LKF->FriVely + a_hy *Input->Time;

	 LKF->NexHea= LKF->FriHea + ((LKF->FriVely*tan(LKF->FriStee)*cos(Angle->AngleBeta))/LENGTH_CAR)*Input->Time;

	 LKF->NexStee= LKF->FriStee + Input->Stee*Input->Time;
// Prediction Covariance
	 float Mul_Result[6][6];
	 float Trans_Result[6][6];
	 float Jacobian[6][6];

	 memset(Trans_Result,0,sizeof(Trans_Result));
	 memset(Jacobian,0,sizeof(Jacobian));

	 Jacobian[0][0]=1;
	 Jacobian[0][1]=0;
	 Jacobian[0][2]=Input->Time*cos(LKF->FriHea+atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR));
	 Jacobian[0][3]=0;
	 Jacobian[0][4]=-LKF->FriVelx*Input->Time*sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR));
	 Jacobian[0][5]=-(LKF->FriVelx *LENGTH_CAR * Input->Time *sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR))*1/cos(LKF->FriStee)*cos(LKF->FriStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee);

	 Jacobian[1][0]=0;
	 Jacobian[1][1]=1;
	 Jacobian[1][2]=0;
	 Jacobian[1][3]=Input->Time*sin(LKF->FriHea+atan((LENGTH_REAR*tan(LKF->FriStee))/LENGTH_CAR));
	 Jacobian[1][4]=LKF->FriVely*sin(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR));
	 Jacobian[1][5]=(LKF->FriVely *LENGTH_CAR * Input->Time *cos(LKF->FriHea+atan(LENGTH_REAR*tan(LKF->FriStee)/LENGTH_CAR))*1/cos(LKF->FriStee)*cos(LKF->FriStee))/LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee);

	 Jacobian[2][0]=0;
	 Jacobian[2][1]=0;
	 Jacobian[2][2]=1;
	 Jacobian[2][3]=0;
	 Jacobian[2][4]=0;
	 Jacobian[2][5]=0;

	 Jacobian[3][0]=0;
	 Jacobian[3][1]=0;
	 Jacobian[3][2]=0;
	 Jacobian[3][3]=1;
	 Jacobian[3][4]=0;
	 Jacobian[3][5]=0;

	 Jacobian[4][0]=0;
	 Jacobian[4][1]=0;
	 Jacobian[4][2]=0;
	 Jacobian[4][3]=(Input->Time*tan(LKF->FriStee)*fabs(LKF->FriStee))/(LKF->FriStee*sqrt(LKF->FriStee*LKF->FriStee+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee)));
	 Jacobian[4][4]=1;
	 Jacobian[4][5]=(LKF->FriVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(LKF->FriStee)*1/cos(LKF->FriStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee),1.5)*fabs(LKF->FriStee));

	 Jacobian[5][0]=0;
	 Jacobian[5][1]=0;
	 Jacobian[5][2]=0;
	 Jacobian[5][3]=0;
	 Jacobian[5][4]=0;
	 Jacobian[5][5]=1;

	 matrixMultiplication(Jacobian,LKF->Prediction_CovarianceFri, Mul_Result);
	 transposeMatrixNxN(Jacobian, Trans_Result);
	 matrixMultiplication(Mul_Result,Trans_Result,LKF->Prediction_CovarianceNex); // Covaricaace result

}
void EFK_GPSHandleMeasurement(GPS *GPS, LKF *LKF ){

	float MeasurementFri[2][2];

	float Inovation[2][2];
	float InovationCov[5][5];

	float GPSMeasurement[2][2];
	float GPSModel[2][5];
	float GPSModelT[5][2];
	float GPSCov[2][2];

	float PredictionNex[2][2];
	float KalmanGian[2][2];
	float Identity[2][2];
//identity
	 Identity[0][0]=1;
	 Identity[1][1]=1;
//GPS covariance
	GPSCov[0][0]=GPS->GPSCovariance[0][0];
	GPSCov[1][1]=GPS->GPSCovariance[1][1];
//GPS measurement
	GPSMeasurement[0][0]=GPS->GPSGetPosition[0][0];
	GPSMeasurement[1][1]=GPS->GPSGetPosition[1][1];
//GPS Modle

	memset(GPSModel,0,sizeof(GPSModel));
	GPSModel[0][0]=GPS->GPS_Model[0][0];
	GPSModel[1][1]=GPS->GPS_Model[1][1];
	transposeMatrixNxM(2,5,GPSModel,GPSModelT);

	//First Measurement
	MeasurementFri[0][0]= LKF->FriPx;
	MeasurementFri[1][1]= LKF->FriPy;
	//prediction
	PredictionNex[0][0]=LKF->Prediction_CovarianceNex[0][0];
	PredictionNex[1][1]=LKF->Prediction_CovarianceNex[1][1];
	//inovation
	Inovation[0][0]= GPSMeasurement[0][0] - MeasurementFri[0][0];
	Inovation[1][1]= GPSMeasurement[1][1] - MeasurementFri[1][1];
	//inovation covariacne
	InovationCov[0][0]= GPSModel[0][0] * PredictionNex[0][0]*GPSModelT[0][0];
	InovationCov[1][1]= GPSModel[1][1] * PredictionNex[1][1]*GPSModelT[1][1];
	InovationCov[0][0]=  InovationCov[0][0] +GPSCov[0][0];
	InovationCov[1][1]=  InovationCov[1][1] + GPSCov[1][1];
	//Kalman Gain
	KalmanGian[0][0]=  PredictionNex[0][0] *  GPSModel[0][0] * InovationCov[0][0];
	KalmanGian[1][1]=  PredictionNex[1][1] *  GPSModel[1][1] * InovationCov[1][1];
	//update
	LKF->FriPx = LKF->NexPx + (KalmanGian[0][0] * Inovation[0][0]);
	LKF->FriPy = LKF->NexPy + (KalmanGian[1][1] * Inovation[1][1]);
	//update covariance
	LKF->Prediction_CovarianceFri[0][0]=(Identity[0][0]-(KalmanGian[0][0] * GPSModel[0][0]))*PredictionNex[0][0];
	LKF->Prediction_CovarianceFri[1][1]=(Identity[1][1]-(KalmanGian[1][1] * GPSModel[1][1]))*PredictionNex[1][1];
}

void Heading_HandleMeasurement(LKF *LKF, Input *Input, Heading *Heading){
	float Jacobian[6][6];
	float JacobianT[6][6];
	float JacobianIn[6][6];
	float Inovation;
	float Measurement[6][6];
	float InovationCov[6][6];
	float InovationCovInv[6][6];
	float InovationResul[6][6];
	float KalmanGain[6][6];
	float KalmanGainb1[6][6];
	float resultb1[6][6];
	float resultb2[6][6];
	float Indentity[6][6];
	float KalmanGiansub[6][6];
	float CovarianceResult[6][6];
	float CovarianceResultb1[6][6];

	Indentity[0][0]=1;
	Indentity[1][1]=1;
	Indentity[2][2]=1;
	Indentity[3][3]=1;
	Indentity[4][4]=1;
	Indentity[5][5]=1;

		 Jacobian[0][0]=0;
		 Jacobian[0][1]=0;
		 Jacobian[0][2]=0;
		 Jacobian[0][3]=(Input->Time*tan(LKF->FriStee)*fabs(LKF->FriStee))/(LKF->FriStee*sqrt(LKF->FriStee*LKF->FriStee+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee)));
		 Jacobian[0][4]=1;
		 Jacobian[0][5]=(LKF->FriVely*LENGTH_CAR*LENGTH_CAR*LENGTH_CAR*Input->Time*(1/cos(LKF->FriStee)*1/cos(LKF->FriStee)))/(pow(LENGTH_CAR*LENGTH_CAR+LENGTH_REAR*LENGTH_REAR*tan(LKF->FriStee)*tan(LKF->FriStee),1.5)*fabs(LKF->FriStee));

	Inovation = Heading->Yaw - LKF->NexHea;

	matrixMultiplication(Jacobian, LKF->Prediction_CovarianceNex, resultb1);
	transposeMatrixNxN(Jacobian, JacobianT);
	matrixMultiplication(resultb1,JacobianT,InovationResul);
	addScalarToMatrix(6, 6, InovationResul,Heading->Covariane);


	matrixMultiplication(LKF->Prediction_CovarianceNex,JacobianT,KalmanGainb1);

	inverseMatrix(Jacobian, JacobianIn);

	matrixMultiplication(JacobianIn,KalmanGainb1,KalmanGain);

	assignMatrix(KalmanGain, KalmanGiansub, 6, 6);

	multiplyMatrixByScalar(KalmanGain, 6, 6, Inovation);

	LKF->FriHea= LKF->NexHea + KalmanGain[0][5];

	matrixMultiplication(KalmanGiansub, Jacobian, CovarianceResult);

	subtractMatrices(Indentity, CovarianceResult, CovarianceResultb1, 6, 6);

	matrixMultiplication(CovarianceResultb1, LKF->Prediction_CovarianceNex,  LKF->Prediction_CovarianceFri);

}
