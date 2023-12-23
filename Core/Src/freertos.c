/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno055.h"
#include "bno_config.h"
#include <stdio.h>
#include "Calib.h"
#include "i2c.h"
#include "usart.h"
#include "CAN_Handle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
	bno055_euler_t eul = { 0, 0, 0 };
	bno055_t bno;
	error_bno err;
	EulerCalib Eulcalib;
	AccCalib AcceCalib;
/* USER CODE END Variables */
/* Definitions for mReadBNO */
osThreadId_t mReadBNOHandle;
const osThreadAttr_t mReadBNO_attributes = {
  .name = "mReadBNO",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mySend_CAN */
osThreadId_t mySend_CANHandle;
const osThreadAttr_t mySend_CAN_attributes = {
  .name = "mySend_CAN",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ReadBNO(void *argument);
void Send_CAN(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	bno = (bno055_t ) { .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU,
								._temp_unit = 0,
						// .ptr = &bno,
						};
		if ((err = bno055_init(&bno)) == BNO_OK) {
			printf("BNO055 success\r\n");
			osDelay (100);
		} else {
			printf("[!] BNO055 failed\r\n");
			printf("%s\n", bno055_err_str(err));
			Error_Handler();
		}
		err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
				BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
		if (err != BNO_OK) {
			printf("[BNO] Failed to set units. Err: %d\r\n", err);
		} else {
			printf("[BNO] Unit selection success\r\n");
		}
		osDelay (100);
		Euler_Calibration(&bno, &eul, &Eulcalib);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mReadBNO */
  mReadBNOHandle = osThreadNew(ReadBNO, NULL, &mReadBNO_attributes);

  /* creation of mySend_CAN */
  mySend_CANHandle = osThreadNew(Send_CAN, NULL, &mySend_CAN_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ReadBNO */
/**
  * @brief  Function implementing the mReadBNO thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ReadBNO */
void ReadBNO(void *argument)
{
  /* USER CODE BEGIN ReadBNO */
  /* Infinite loop */
  for(;;)
  {
			Euler_getData(&eul, &bno, &Eulcalib);
			printf("Y %+2.2f\r\n",eul.yaw);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			osDelay(1);
  }
  /* USER CODE END ReadBNO */
}

/* USER CODE BEGIN Header_Send_CAN */
/**
* @brief Function implementing the mySend_CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_CAN */
void Send_CAN(void *argument)
{
  /* USER CODE BEGIN Send_CAN */
	uint32_t ID =0x65;
	uint8_t Data[8]={1,2,3,4,5,6,7,8};
	uint8_t size =0;
  /* Infinite loop */
  for(;;)
  {
	CAN_HandleSenđata(ID,Data,size);
    osDelay(50);
  }
  /* USER CODE END Send_CAN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

