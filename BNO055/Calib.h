/*
 * Calib.h
 *
 *  Created on: Nov 22, 2023
 *      Author: win 10
 */

#ifndef CALIB_H_
#define CALIB_H_

typedef struct{
	float RateRoll;
	float RatePitch;
	float RateYaw;
}EulerCalib;

typedef struct{
	float RateAccx;
	float RateAccy;
	float RateAccz;
}AccCalib;
void Euler_Calibration(bno055_t *bno , bno055_euler_t *eul,EulerCalib *Euler);
void Euler_getData(bno055_euler_t *eul,bno055_t *bno,EulerCalib *Euler);
void ACC_getData(bno055_vec3_t *acc, bno055_t *bno,AccCalib *Acc);
void ACC_Calibration(bno055_t *bno , bno055_vec3_t *acc,AccCalib *Acc);


#endif /* CALIB_H_ */
