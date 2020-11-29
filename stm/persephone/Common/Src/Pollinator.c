/*
 * Pollinator function
 *
 * By: Josh Panos
 *
  */

#include <stm32h7xx_hal.h>
#include "Pollinator.h"


	// static unsigned short Touch_Button = 0;
	// int angle; //1 means neg 45 degree  3 means 45 degree

int check_sense(void){
	int Touch_Button = ((GPIOB->IDR & GPIO_IDR_ID8) == GPIO_IDR_ID8);
	return Touch_Button;
}
void set_angle(int angle){
	TIM1->CCR1 = angle;
}

void init_touchISR(void) {
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE8); //Touch sensor input
	GPIOB->MODER |= GPIO_MODER_MODE8_1;
	GPIOB->AFR[1] &= ~(0xf);
	GPIOB->AFR[1] |= 1; // set tim1_ch1

	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	TIM16->CR1 |= TIM_CR1_ARPE; // set auto reload preload
	TIM16->CR1 &= ~(TIM_CR1_DIR); // set counter direction up
	TIM16->PSC = 64000 - 1;
	TIM16->ARR = 50 - 1;

	// set input to tim16_ch1 input
	TIM16->TISEL &= ~(TIM_TISEL_TI1SEL);
	// set ccr1 to input; set cc1s to 01 to configure as input
	TIM16->CCMR1 &= ~(TIM_CCMR1_CC1S);
	TIM16->CCMR1 |= TIM_CCMR1_CC1S_0;
	// configure input filter
	TIM16->CCMR1 &= ~(TIM_CCMR1_IC1F);
	TIM16->CCMR1 |= (TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_0);
	// set to rising edge detection
	TIM16->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);
	// set input prescaler to zero
	TIM16->CCMR1 &= ~(TIM_CCMR1_IC1PSC);
	// enable capture of timer into register
	TIM16->CCER |= TIM_CCER_CC1E;
	// enable interrupt generation
	TIM16->DIER |= TIM_DIER_CC1IE;

	TIM16->CR1 |= TIM_CR1_CEN;
}

void init_servoPWM(void)
{

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOEEN;

	GPIOE->MODER &= ~(GPIO_MODER_MODE9);
	GPIOE->MODER |= GPIO_MODER_MODE9_1; // set alternate function
	GPIOE->AFR[1] &= ~(0xf << 4);
	GPIOE->AFR[1] |= (1 << 4); // set tim1_ch1
	GPIOE->OTYPER |= GPIO_OTYPER_OT9; // set open drain
	// GPIOE->OTYPER &= ~GPIO_OTYPER_OT9;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	TIM1->CR1 |= TIM_CR1_ARPE; // set auto reload preload
	TIM1->CR1 &= ~(TIM_CR1_DIR); // set counter direction up
	TIM1->PSC = 64 - 1;
	TIM1->ARR = 20000 -1;

	TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S); // set to output
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE; // preload enable (idk requried for pwm ref manual says)
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M);
	TIM1->CCMR1 |= 0x6 << TIM_CCMR1_OC1M_Pos; // pwm mode 1

	TIM1->CCER |= TIM_CCER_CC1E; // enable channel 1 output
	TIM1->BDTR |= TIM_BDTR_MOE; // enable master output

	TIM1->EGR |= TIM_EGR_UG; //update generation so that all values are loaded into shadow registers

	TIM1->CR1 |= TIM_CR1_CEN;

}

