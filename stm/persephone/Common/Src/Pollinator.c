/*
 * Pollinator function
 *
 * By: Josh Panos
 *
  */

#include "main.h"
#include "Pollinator.h"


	static unsigned short Touch_Button = 0;
	int angle; //1 means neg 45 degree  3 means 45 degree


void init_GPIO(void){
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE8);
	GPIOA->MODER |= GPIO_MODER_MODE8_1; // set alternate function
	GPIOA->AFR[1] &= ~(0xf);
	GPIOA->AFR[1] |= 1; // set tim1_ch1
	GPIOA->OTYPER |= GPIO_OTYPER_OT8; // set open drain

	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE8); //Touch sensor input
}

void init_TIM(int angle)
{

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

	while(1){

		Touch_Button = ((GPIOB->IDR &= GPIO_IDR_ID8) == (1 >> 8));
		if(Touch_Button)
		{
			TIM1->CCR1 = 2;
		}
		else
		{
			TIM1->CCR1 = angle;
		}
	}


}

