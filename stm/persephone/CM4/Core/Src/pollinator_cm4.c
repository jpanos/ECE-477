/*
 * pollinator_cm4.c
 *
 *  Created on: Nov 28, 2020
 *      Author: ethan
 */
#include <pollinator_cm4.h>
#include <Pollinator.h>
#include <stm32h7xx_hal.h>

uint8_t _pollinator_jig_count;
void TIM16_IRQHandler() {
	if (TIM16->SR & TIM_SR_CC1IF) {
		uint16_t val = TIM16->CCR1;
		GPIOB->ODR ^= GPIO_ODR_OD0;

		_pollinator_jig_count = 0;
		set_angle(POLLINATOR_DOWN_ANGLE);
		TIM16->DIER |= TIM_DIER_UIE;
	}
	else { // when UIE enabled, interrupt fires a 20Hz
		TIM16->SR &= ~TIM_SR_UIF;
		_pollinator_jig_count++;
		switch (_pollinator_jig_count) {
		  case 5:
		    set_angle_deg(POLLINATOR_UP_ANGLE);
		    break;
		  case 10:
		    set_angle_deg(POLLINATOR_DOWN_ANGLE);
		    break;
		  case 15:
		    set_angle_deg(POLLINATOR_UP_ANGLE);
		    break;
		  case 20:
	      TIM16->DIER &= ~TIM_DIER_UIE;
	      set_angle_deg(POLLINATOR_STRAIGHT_ANGLE);
		    break;
		  default:
		    set_angle_deg(POLLINATOR_STRAIGHT_ANGLE);
		    break;
		}
	}
}

void init_pollinator() {
	// touch edge detection interrupt
	init_servoPWM();
	set_angle_deg(POLLINATOR_STRAIGHT_ANGLE);
	init_touchISR();
	NVIC_EnableIRQ(TIM16_IRQn);
	// init green LED for test
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE0);
	GPIOB->MODER |= GPIO_MODER_MODE0_0;
	GPIOB->ODR &= ~GPIO_ODR_OD0;
}
