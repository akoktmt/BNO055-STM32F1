/*
 * Calib.c
 *
 *  Created on: Nov 22, 2023
 *      Author: win 10
 */
#include "bno055.h"
#include "bno_config.h"
#include "Calib.h"
void Euler_Calibration(bno055_t *bno , bno055_euler_t *eul,EulerCalib *Euler){
		float RateRoll,RatePitch,RateYaw;
		int RateNumber;
		bno->euler(bno, eul);
		for(RateNumber=0;RateNumber<2000;RateNumber++)
		{
		RateRoll+=eul->roll;
		RatePitch+=eul->pitch;
		RateYaw+=eul->yaw;
		HAL_Delay(1);
		}
		RateRoll/=2000;
		RatePitch/=2000;
		RateYaw/=2000;

		Euler->RatePitch=RatePitch;
		Euler->RateRoll=RateRoll;
		Euler->RateYaw=RateYaw;
}
void Euler_getData(bno055_euler_t *eul,bno055_t *bno,EulerCalib *Euler){

				bno->euler(bno, eul);
			    eul->roll-= Euler->RateRoll;
				eul->pitch-=Euler->RatePitch;
				eul->yaw-=Euler->RateYaw;
}

void ACC_Calibration(bno055_t *bno , bno055_vec3_t *acc,AccCalib *Acc){
		float RateAccx,RateAccy,RateAccz;
		int RateNumber;
		bno->acc(bno, acc);
		for(RateNumber=0;RateNumber<2000;RateNumber++)
		{
			RateAccx+=acc->x;
			RateAccy+=acc->y;
			RateAccz+=acc->z;
		HAL_Delay(1);
		}
		RateAccx/=2000;
		RateAccy/=2000;
		RateAccz/=2000;

		Acc->RateAccx=RateAccx;
		Acc->RateAccy=RateAccy;
		Acc->RateAccz=RateAccz;
}
void ACC_getData(bno055_vec3_t *acc, bno055_t *bno,AccCalib *Acc){

				bno->acc(bno, acc);
				acc->x-= Acc->RateAccx;
				acc->y-=Acc->RateAccy;
				acc->z-=Acc->RateAccz;
}
