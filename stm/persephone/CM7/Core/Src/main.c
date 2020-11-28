/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Pollinator.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HSEM_ID_0 (0U) /* HW semaphore 0*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	//mavlink_message_t msg;
  /* USER CODE END 1 */

	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	shared->mav_state = MAV_STATE_UNINIT;
  int32_t timeout;
  /* USER CODE END Boot_Mode_Sequence_0 */

  /* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
  /* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	// mavlink_initialize();

	// initialize push button stuff
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
	GPIOC->MODER &= ~(0xc000000);
	GPIOB->ODR ^= GPIO_ODR_OD14;

	// put yellow led in output mode
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
	GPIOE->MODER &= ~(GPIO_MODER_MODE1);
	GPIOE->MODER |= GPIO_MODER_MODE1_0;
	GPIOE->ODR &= ~GPIO_ODR_OD1;

	//  /* USER CODE END 2 */

	//  /* Infinite loop */
	//  /* USER CODE BEGIN WHILE */
  while (shared->mav_state == MAV_STATE_UNINIT) {}
	uint8_t prev_val;
	mavlink_message_t takeoff_msg;


	while (1)
	{
		// a wait for loop used for my shitty edge detection
		for (int i = 0; i <1000000; i++){}
		if (prev_val == 0 && GPIOC->IDR != 0) {
			float alt = shared->altitude_msl;
			// yellow light on to show that in if statement
			GPIOE->ODR |= GPIO_ODR_OD1;
			msleep(5000);

			// enable mode; not sure if this is needed
			send_command_long(MAV_CMD_NAV_GUIDED_ENABLE, 0, 0, 0, 0, 0, 0, 0);
			// set to offboard mode
			set_offboard(0);
			// a variabe, sets z setpoint to halz z velocit at
			float z_setpoint = shared->pos_z - 1.25;
			// set z velocity to -.7 m/s (z positive axis is down)
			set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, 0, 0, -.7, 0, 0, 0, 0, 0);
			// arm drone
			send_arm_disarm_message(1, 0);
			msleep(500);

			// wait for z pos to reach above setpoint
			while (shared->pos_z > z_setpoint) {}
			// turn off yellow led
			GPIOE->ODR &= ~GPIO_ODR_OD1;
			// 3rd argument is mask, when set to 0x1000 or 0x2000, puts drone in loiter mode
			set_hold2(0);
			msleep(5000);
			// set_offboard(0);

			// mav mask reference: yaw_rate yaw force afz | afy afx vz vy | vx z y x

			// test 1: set velocity forward LOCAL frame
			// set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, .5, 0, 0, 0, 0, 0, 0, 0);
			// msleep(4000);

			// test 2: set velocity forward BODY frame
			set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, .5, 0, 0, 0, 0, 0, 0, 0);
			msleep(10000);

			// float x_setpoint = shared->pos_x + 2;

			// test 3: set position forward LOCAL frame
			// set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_POSITION_SETPOINT, x_setpoint, shared->pos_set_y, shared->pos_set_z, 0, 0, 0, 0, 0, 0, 0, 0);
			// msleep(5000);
			// while (shared->pos_x > x_setpoint) {}
			//msleep(4000);

			// test 4: set position forward BODY frame
			// set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_POSITION_SETPOINT, x_setpoint, shared->pos_set_y, shared->pos_set_z, 0, 0, 0, 0, 0, 0, 0, 0);
			// msleep(5000);
			// while (shared->pos_x < x_setpoint) {}

			// test 5: set position and velocity forward LOCAL frame
			// set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, 0xdc4, x_setpoint, 0, 0, .5, 0, 0, 0, 0, 0, 0, 0);
			// while (shared->pos_x < x_setpoint) {}

			// test 6: set position and velocity forward BODY frame
			// set_pos_setpoint(0, MAV_FRAME_BODY_NED, 0xdc4, x_setpoint, 0, 0, .5, 0, 0, 0, 0, 0, 0, 0);
			// while (shared->pos_x < x_setpoint) {}

			set_hold2(0);
			msleep(5000);
			// set_offboard(0);

			// set setpoint to go down at .7 m/s
			set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, 0, 0, .7, 0, 0, 0, 0, 0);

			// when landed state is detected, turn on led and disarm drone
			while (shared->landed_state != MAV_LANDED_STATE_ON_GROUND);
			GPIOE->ODR |= GPIO_ODR_OD1;
			send_arm_disarm_message(0, 0);
		}
		// other part of shitty edge detection
		prev_val = GPIOC->IDR >> 8;
    /* USER CODE END WHILE */
    
		/* USER CODE BEGIN 3 */

		if(check_sense())
		{
			set_angle(2);
		}
		else
		{
			set_angle(3);
		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void TIM16_IRQHandler() {
	uint16_t val = TIM16->CCR1;
	GPIOB->ODR ^= GPIO_ODR_OD0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
