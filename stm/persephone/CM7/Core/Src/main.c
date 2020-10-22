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


	static unsigned short User_Button = 0;
	static unsigned short Touch_Button = 0;


	//Initialize GPIO Pins
	//GPIO A
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE8);
	GPIOA->MODER |= GPIO_MODER_MODE8_1; // set alternate function
	GPIOA->AFR[1] &= ~(0xf);
	GPIOA->AFR[1] |= 1; // set tim1_ch1
	GPIOA->OTYPER |= GPIO_OTYPER_OT8; // set open drain

	//GPIO B

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODE0); //LED1
	GPIOB->MODER |= GPIO_MODER_MODE0_0;

	GPIOB->MODER &= ~(GPIO_MODER_MODE14); //LED3
	GPIOB->MODER |= GPIO_MODER_MODE14_0;
	GPIOB->MODER &= ~(GPIO_MODER_MODE8); //Touch sensor input
	//GPIOB->ODR 	|= GPIO_ODR_OD14;

	//GPIO C

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);

	//GPIO E

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;

	GPIOE->MODER &= ~(GPIO_MODER_MODE1); //LED 3
	GPIOE->MODER |= GPIO_MODER_MODE1_0;
	//GPIOE->ODR 	|= GPIO_ODR_OD1;

	//GPIO F
	//TIM17 CH1 OUTPUT ENABLE
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOFEN;
	/*GPIOF->MODER &= ~(GPIO_MODER_MODE7);
	GPIOF->MODER |= GPIO_MODER_MODE7_1;
	GPIOF->AFR[1] &= ~(0xf);
	GPIOF->AFR[1] |= 1;
	GPIOF->OTYPER |= GPIO_OTYPER_OT7;*/
	GPIOF->MODER &= ~(GPIO_MODER_MODE7);
	GPIOF->MODER |= GPIO_MODER_MODE7_1; // set alternate function
	GPIOF->AFR[0] &= ~(0xf << 28);
	GPIOF->AFR[0] |= 1 << 28; // set tim1_ch1
	GPIOF->OTYPER |= GPIO_OTYPER_OT7; // set open drain

	//TIM1

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CR1 |= TIM_CR1_ARPE; // set auto reload preload
	TIM1->CR1 &= ~(TIM_CR1_DIR); // set counter direction up
	TIM1->PSC = 32000 - 1;
	TIM1->ARR = 40 -1;

	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S); // set to output
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // preload enable (idk requried for pwm ref manual says)
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM1->CCMR1 |= 0x6 << TIM_CCMR1_OC1M_Pos; // pwm mode 1

	TIM1->CCER |= TIM_CCER_CC1E; // enable channel 1 output
	TIM1->BDTR |= TIM_BDTR_MOE; // enable master output

	TIM1->EGR |= TIM_EGR_UG; //update generation so that all values are loaded into shadow registers

	TIM1->CR1 |= TIM_CR1_CEN;

	//Config for MCO1

	//RCC->CFGR |= RCC_CFGR_MCO1_2;
	//RCC->CFGR |= (RCC_CFGR_MCO1PRE_3 | RCC_CFGR_MCO1PRE_2);


	// Tim 17 stuff
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	TIM17->CR1 |= TIM_CR1_ARPE; // set auto reload preload
	TIM17->CR1 &= ~(TIM_CR1_DIR); // set counter direction up
	TIM17->PSC = 1000 - 1;
	TIM17->ARR = 2 -1;

	TIM17->CCMR1 &= ~(TIM_CCMR1_CC1S); // set to output
	TIM17->CCMR1 |= TIM_CCMR1_OC1PE; // preload enable (idk requried for pwm ref manual says)
	TIM17->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM17->CCMR1 |= 0x6 << TIM_CCMR1_OC1M_Pos; // pwm mode 1
	TIM17->CCR1 = 1; // compare register for pwm (when counter is below this value, pwm signal is high)
	TIM17->CCER |= TIM_CCER_CC1E; // enable channel 1 output
	TIM17->BDTR |= TIM_BDTR_MOE; // enable master output

	TIM17->EGR |= TIM_EGR_UG; //update generation so that all values are loaded into shadow registers

	TIM17->CR1 |= TIM_CR1_CEN;



	// mavlink_initialize();
	// set_mavlink_msg_interval(MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS, 10000);

	//  /* USER CODE END 2 */
	//
	static unsigned short pin_state = 0;
	static unsigned short pin_state1 = 0;
	static unsigned short pin_state2 = 0;
	static unsigned short pin_state3 = 0;


	//  /* Infinite loop */
	//  /* USER CODE BEGIN WHILE */
	while (1)
	{

		/* USER CODE BEGIN 3 */

		User_Button =  ((GPIOC->IDR &= GPIO_IDR_ID13) == (1 >> 13));
		Touch_Button = ((GPIOB->IDR &= GPIO_IDR_ID8) == (1 >> 8));
		if(Touch_Button)
		{
			GPIOB->ODR 	&= (pin_state << 0);
			GPIOE->ODR 	&= (pin_state << 1);
			HAL_Delay(40);
			pin_state = !pin_state;
			GPIOB->ODR 	|= (pin_state << 0);
			HAL_Delay(40);
			pin_state1 = !pin_state1;
			GPIOB->ODR 	|= (pin_state1 << 14);
			HAL_Delay(30);
			pin_state2 = !pin_state2;
			GPIOE->ODR 	|= (pin_state2 << 1);
			HAL_Delay(5);
			TIM1->CCR1 = 2; // compare register for pwm (when counter is below this value, pwm signal is high)
		}
		else
		{
			GPIOB->ODR 	&= (pin_state << 0);
			GPIOE->ODR 	&= (pin_state << 1);
			HAL_Delay(10);
			pin_state = !pin_state;
			GPIOB->ODR 	|= (pin_state << 0);
			HAL_Delay(4);
			pin_state1 = !pin_state1;
			GPIOB->ODR 	|= (pin_state1 << 14);
			HAL_Delay(3);
			pin_state2 = !pin_state2;
			GPIOE->ODR 	|= (pin_state2 << 1);
			HAL_Delay(5);
			TIM1->CCR1 = 1; // compare register for pwm (when counter is below this value, pwm signal is high)
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
