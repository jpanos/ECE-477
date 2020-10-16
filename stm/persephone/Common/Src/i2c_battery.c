#include "i2c_battery.h"

void initI2C(){
	// This function initializes I2C
	uint32_t OwnAddr = 0x52; // devide own address

	//Enable I2C clock and select sysclock as clock source
	RCC->APB1LENR |= RCC_APB1LENR_I2C1EN;
	RCC-> D2CCIP2R |= 0x3<<12; // set clock to CSI clock ~ 4 MHz(Pg 447 ref manual)


	//I2C CR1 config
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 &= ~I2C_CR1_ANFOFF; //en analog filter
	I2C1->CR1 &= ~I2C_CR1_DNF; // disable dig filters
	I2C1->CR1 |= I2C_CR1_ERRIE; // allow error interrupts
	I2C1->CR1 &= ~ I2C_CR1_NOSTRETCH; // allow clock stretching

	// Timing config
	I2C1->TIMINGR = 0; // clear timing reg
	I2C1->TIMINGR &= ~I2C_TIMINGR_PRESC; //clear presc
	I2C1->TIMINGR |= 9<<28; // set presc to 9 (now clock is 400 kHz)
	I2C1->TIMINGR |= 200; // set SCLL to 5
	I2C1->TIMINGR |= 200<<8; // set SCLH to 5
	I2C1->TIMINGR |= 0x2<<16; // set SDADEL to table value
	I2C1->TIMINGR |= 0x4<<20; // set scaldel to table value

	// set i2c own address register
	I2C1->OAR1 &= ~I2C_OAR1_OA1EN;
	I2C1->OAR1 |= I2C_OAR1_OA1EN|OwnAddr; // set the own address
	I2C1->OAR2 &=~I2C_OAR2_OA2EN; // disable oar 2

	// i2c cr2 config
	I2C1->CR2 &= ~ I2C_CR2_ADD10; // do 7 bit addressing
	I2C1->CR2 |= I2C_CR2_AUTOEND; // en autoend
	I2C1->CR1 |= I2C_CR1_PE; // enable peripheral

} // end initI2C

void I2C1GPIOINIT(){
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; // enable clock to gpiob

	GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7); // clear mode regs
	GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1; // make them alternate functions
	GPIOB->OTYPER |= 3<<6; // make open drain pins
	GPIOB->AFR[0] &= ~(0xFF<<24);
	GPIOB->AFR[0] |= 4<<28; // set to i2c
	GPIOB->AFR[0] |= 4<<24; // set to i2c
}

void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction) {
	I2C->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD|I2C_CR2_RD_WRN|I2C_CR2_ADD10);
	if (Direction) // requests a read transfer
	{
		I2C->CR2 |= I2C_CR2_RD_WRN;
	}
	I2C->CR2 |= DevAddress <<1;
	I2C->CR2 |= Size<<16;
}
/* USER CODE END 0 */
