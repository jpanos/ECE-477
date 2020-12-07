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
#include "i2c_battery.h"
#include <shared.h>
#include <spin_lock.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MASTERWRITE 0
#define MASTERREAD  1
#define SYS_STAT 0x00
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	char slaverxdata = 0; // create a global receive data variable
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void I2C2_EV_IRQHandler(void) { // I2C2 interrupt handler
	if((I2C2->ISR & I2C_ISR_RXNE)==I2C_ISR_RXNE){ // is reciever not empty
		shared->masterrxdata = I2C2->RXDR; // read th recieved data
		I2C2->CR1 &= ~I2C_CR1_RXIE;
	}
	else if((I2C2->ISR & I2C_ISR_TXE) == I2C_ISR_TXE){ // need to transmit??
		if (shared->i2cTargReg != -1)
		{
			// send the target register address
			I2C2->CR1 &= ~I2C_CR1_TXIE;
			I2C2->TXDR = shared->i2cTargReg; // send the target reg addr
			shared->i2cTargReg = -1;
			if (shared->i2cmode == MASTERREAD){
				 // dont allow tx interrupt anymore
				I2C2->CR1 |= I2C_CR1_RXIE;
				I2C2battTalk(MASTERREAD, -1, 0x0);

				//I2C2->CR1 |= I2C_CR1_RXIE; // allow rx not empty interrupt
			}
		}
		else {
			I2C2->TXDR = shared->mastertxdata;
			I2C2->CR1 &= ~I2C_CR1_TXIE; // dont allow tx interrupt anymore
		}
	}

}

void USART3_IRQHandler(void) { // uart 3 interrupt handler
	// rx not empty
	// x,y,z coordinates in floats, 12 bytes per packet
	// todo: check endianness!!!!!
	if ((USART3->ISR & (USART_ISR_RXNE_RXFNE))==(USART_ISR_RXNE_RXFNE)){
			USART3->CR1 |= USART_CR1_RTOIE; // enable interrupt for rx timeout
			shared->usartbuff[shared->usartct] = USART3->RDR; // get the byte
			shared->usartct = shared->usartct+1;
		}
	else if ((USART3->ISR & USART_ISR_RTOF)== USART_ISR_RTOF) { // receive timeout
			USART3->ICR |= USART_ICR_RTOCF; // clear the flag.
			USART3->CR1 &=~USART_CR1_RTOIE;
			shared->usartct = 0; // reset the byte count.
			// parse the buffer.
			volatile uint8_t * structaddr = (uint8_t *) &(shared->flowercoord);
			int size = sizeof(Point);

			// spin_lock(HSEM_ID_FLOWER_POS_DATA, UART3_RX_PROC_ID);
			for (int k = 0; k < size; k++){
				structaddr[k] = shared->usartbuff[k];
			}
			// lock_release(HSEM_ID_FLOWER_POS_DATA, UART3_RX_PROC_ID);
			set_flower_setpoint(UART3_RX_PROC_ID);
		}
	USART3->ICR |= 0x123bbf;
}

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

	initI2C2(); 				// init i2c2
	I2C2GPIOINIT();
	initUART();

	// initialize push button stuff
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;
	GPIOC->MODER &= ~(GPIO_MODER_MODE13);
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
  set_vel_hold(0);
	uint8_t prev_val;
	mavlink_message_t takeoff_msg;


	while (1)
	{
		// a wait for loop used for my shitty edge detection
		for (int i = 0; i <1000000; i++){}
		if ((prev_val == 0) && (GPIOC->IDR & GPIO_IDR_ID13)) {
			float alt = shared->altitude_msl;
			// yellow light on to show that in if statement
			GPIOE->ODR |= GPIO_ODR_OD1;
			msleep(5000);

			// enable mode; not sure if this is needed
			send_command_long(MAV_CMD_NAV_GUIDED_ENABLE, 0, 0, 0, 0, 0, 0, 0);
			// set to offboard mode
			set_offboard(0);
			// a variabe, sets z setpoint to halz z velocit at
			shared->idle_height = shared->pos_z - .5;
//			shared->idle_height = shared->altitude_msl + .5;
			// set z velocity to -.7 m/s (z positive axis is down)
			set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, 0, 0, -.7, 0, 0, 0, 0, 0);
			// arm drone
			send_arm_disarm_message(1, 0);
			msleep(500);

			// wait for z pos to reach above setpoint
			while (shared->pos_z > shared->idle_height) {}
//			while (shared->altitude_msl < shared->idle_height) {}
			// turn off yellow led
			GPIOE->ODR &= ~GPIO_ODR_OD1;
			// 3rd argument is mask, when set to 0x1000 or 0x2000, puts drone in loiter mode
			set_vel_hold(0);
//			msleep(5000);

			// spin around until receive flower positional data
			set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VEL_YAWRATE_SETPOINT,
			    0, 0, 0, 0, 0, -.1, 0, 0, 0, 0, -.2);
//			msleep(4000);
			while (shared->flowercoord.x == 0) {
			  float vz = 0;
			  if (shared->pos_z > shared->idle_height + .1) vz = -.1;
	      set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VEL_YAWRATE_SETPOINT,
	          0, 0, 0, 0, 0, vz, 0, 0, 0, 0, -.2);
			}
      set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VEL_YAWRATE_SETPOINT,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      shared->pos_mode |= MVPSSC_POS_MODE_FLOWER;
      GPIOE->ODR ^= GPIO_ODR_OD1;
//      msleep(3000);
      while (!(shared->pos_mode & MVPSSC_POS_MODE_LAND)) {}
//      msleep(10000);
//
//			set_vel_hold(0);
//			msleep(5000);
//
//			// spin around until receive flower positional data
//			set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VEL_YAWRATE_SETPOINT,
//			    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, .4);
//			while (shared->flowercoord.x == 0) {}
//      set_pos_setpoint(0, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VEL_YAWRATE_SETPOINT,
//          0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//      shared->pos_mode |= MVPSSC_POS_MODE_FLOWER;
//      msleep(10000);

//			set_vel_hold(0);
//			msleep(5000);
			// set_offboard(0);

			// set setpoint to go down at .7 m/s
			set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, 0, 0, .7, 0, 0, 0, 0, 0);

			// when landed state is detected, turn on led and disarm drone
			while (shared->landed_state != MAV_LANDED_STATE_ON_GROUND);
			GPIOE->ODR |= GPIO_ODR_OD1;
			send_arm_disarm_message(0, 0);
		}
		// other part of shitty edge detection
		prev_val = (GPIOC->IDR & GPIO_IDR_ID13) >> 8;
		/* USER CODE BEGIN 3 */

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
