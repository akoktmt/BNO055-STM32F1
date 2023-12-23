/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "bno055.h"
//#include "bno_config.h"
#include <stdio.h>
#include "CAN_Handle.h"
//#include "ssd1306.h"
//#include "fonts.h"
//#include "test.h"
//#include "Calib.h"
//#include "LKF.h"
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

/* USER CODE BEGIN PV */
//bno055_t bno;
//error_bno err;
//EulerCalib Eulcalib;
//AccCalib AcceCalib;
//LKF KF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  CAN_Config_filtering();
  if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_TX_MAILBOX_EMPTY|CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK){
  	 printf("Start CAN IT Error");
   }
    if(HAL_CAN_Start(&hcan)!=HAL_OK){
  	 printf("Start CAN Error");
   }
	//SSD1306_Init();
//	bno = (bno055_t ) { .i2c = &hi2c1, .addr = BNO_ADDR, .mode = BNO_MODE_IMU,
//					._temp_unit = 0,
//			// .ptr = &bno,
//			};
//	HAL_Delay(1000);
//	if ((err = bno055_init(&bno)) == BNO_OK) {
//		printf("[+] BNO055 init success\r\n");
//		HAL_Delay(100);
//	} else {
//		printf("[!] BNO055 init failed\r\n");
//		printf("%s\n", bno055_err_str(err));
//		Error_Handler();
//	}
//	HAL_Delay(100);
//	err = bno055_set_unit(&bno, BNO_TEMP_UNIT_C, BNO_GYR_UNIT_DPS,
//			BNO_ACC_UNITSEL_M_S2, BNO_EUL_UNIT_DEG);
//	if (err != BNO_OK) {
//		printf("[BNO] Failed to set units. Err: %d\r\n", err);
//	} else {
//		printf("[BNO] Unit selection success\r\n");
//	}
//
//	HAL_Delay(1000);
	// f32 acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
//	bno055_vec3_t acc = { 0, 0, 0 };
//	bno055_vec3_t lia = { 0, 0, 0 };
//	bno055_vec3_t gyr = { 0, 0, 0 };
//	bno055_vec3_t mag = { 0, 0, 0 };
//	bno055_vec3_t grv = { 0, 0, 0 };
//	bno055_euler_t eul = { 0, 0, 0 };
//	bno055_vec4_t qua = { 0, 0, 0 };
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//Euler_Calibration(&bno, &eul, &Eulcalib);
//ACC_Calibration(&bno, &acc, &AcceCalib);
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//			bno.temperature(&bno, &temperature);
//			bno.acc(&bno, &acc);
//			bno.linear_acc(&bno, &lia);
//			bno.mag(&bno, &mag);
//			bno.gyro(&bno, &gyr);
//			bno.gravity(&bno, &grv);
//			bno.quaternion(&bno, &qua);
//		        printf("[+] Temperature: %2d°C\r\n", temperature);
//		ACC_getData(&acc, &bno, &AcceCalib);
//		Euler_getData(&eul, &bno, &Eulcalib);
//		printf("Y %+2.2f\r\n",eul.yaw);
//		        printf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y,
//		               lia.z);
//		        printf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y,
//		               gyr.z);
//		        printf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y,
//		               mag.z);
//		        printf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y,
//		               grv.z);
//				Euler_getData(&eul, &bno, &Eulcalib);
//		        printf("[+] Roll: %+2.2f | Pitch: %+2.2f | Yaw: %+2.2f\r\n", eul.roll,
//		        		eul.pitch,eul.yaw);
//		        printf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n",
//		               qua.w, qua.x, qua.y, qua.z);
//		snprintf(buffer, sizeof(buffer), "%s%2d°C", "Temperature:", temperature);
//		 printf("[+] Temperature: %2d°C\r\n", temperature);
//		        printf("[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y,
//		               acc.z);
//		        printf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y,
//		               lia.z);
//		        printf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y,
//		               gyr.z);
//		        printf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y,
//		               mag.z);
//		        printf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y,
//		               grv.z);
//		        printf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n",
//		               qua.w, qua.x, qua.y, qua.z);
//		SSD1306_GotoXY(5, 40);
//		SSD1306_Puts(buffer, &Font_7x10, 1);
//		printf("[+] ACC - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", acc.x, acc.y,
//				acc.z);
//		printf("[+] LIA - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", lia.x, lia.y,
//				lia.z);
//		printf("[+] GYR - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", gyr.x, gyr.y,
//				gyr.z);
//		printf("[+] MAG - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", mag.x, mag.y,
//				mag.z);
//		printf("[+] GRV - x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n", grv.x, grv.y,
//				grv.z);
//		snprintf(buffer, sizeof(buffer), "%s%+2.2f", "R ", eul.roll);
//		SSD1306_GotoXY(5, 4);
//		SSD1306_Puts(buffer, &Font_7x10, 1);
//		snprintf(buffer, sizeof(buffer), "%s%+2.2f", "Y ", eul.yaw);
//		SSD1306_GotoXY(5, 16);
//		SSD1306_Puts(buffer, &Font_7x10, 1);
//		snprintf(buffer, sizeof(buffer), "%s%+2.2f", "Pitch:", eul.pitch);
//		SSD1306_GotoXY(5, 26);
//		SSD1306_Puts(buffer, &Font_7x10, 1);
//		SSD1306_UpdateScreen(); // update screen
//		printf("[+] QUA - w: %+2.2f | x: %+2.2f | y: %+2.2f | z: %+2.2f\r\n",
//				qua.w, qua.x, qua.y, qua.z);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(100);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
