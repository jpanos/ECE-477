#include "i2c_battery.h"


void initI2C2(void){
	// This function initializes I2C
	uint32_t OwnAddr = 0x52; // decide own address
	//Enable I2C clock and select sysclock as clock source
	RCC->APB1LENR |= RCC_APB1LENR_I2C2EN;
	//RCC-> D2CCIP2R |= 0x3<<12; // set clock to CSI clock ~ 4 MHz(Pg 447 ref manual) (todo: check)

	//I2C CR1 config
	I2C2->CR1 &= ~I2C_CR1_PE;
	I2C2->CR1 &= ~I2C_CR1_ANFOFF; //en analog filter
	I2C2->CR1 &= ~I2C_CR1_DNF; // disable dig filters
	I2C2->CR1 |= I2C_CR1_ERRIE; // allow error interrupts
	I2C2->CR1 &= ~ I2C_CR1_NOSTRETCH; // allow clock stretching

	// Timing config
	I2C2->TIMINGR = 0; // clear timing reg
	I2C2->TIMINGR &= ~I2C_TIMINGR_PRESC; //clear presc
	I2C2->TIMINGR |= 9<<28; // set presc to 9 (now clock is 400 kHz)
	I2C2->TIMINGR |= 32; // set SCLL to 5
	I2C2->TIMINGR |= 26<<8; // set SCLH to 5
	I2C2->TIMINGR |= 0x2<<16; // set SDADEL to table value
	I2C2->TIMINGR |= 0x4<<20; // set scaldel to table value

	// set i2c own address register
	I2C2->OAR1 &= ~I2C_OAR1_OA1EN;
	I2C2->OAR1 |= I2C_OAR1_OA1EN|OwnAddr; // set the own address
	I2C2->OAR2 &=~I2C_OAR2_OA2EN; // disable oar 2

	// i2c cr2 config
	I2C2->CR2 &= ~ I2C_CR2_ADD10; // do 7 bit addressing
	I2C2->CR2 |= I2C_CR2_AUTOEND; // en autoend
	I2C2->CR1 |= I2C_CR1_PE; // enable peripheral
} // end initI2C

void initI2C1(void) {  // configures i2c1 as the slave (pg 2136 of ref manual)
//Enable I2C clock and select sysclock as clock source
	uint32_t slaveOwnAddr = 0x30; // set the slave address
	NVIC_EnableIRQ(I2C1_EV_IRQn);  // enable I2C1 event interrupt

	RCC->APB1LENR |= RCC_APB1LENR_I2C1EN;

	//I2C CR1 config
	I2C1->CR1 &= ~I2C_CR1_PE;
	I2C1->CR1 &= ~I2C_CR1_ANFOFF; //en analog filter
	I2C1->CR1 &= ~I2C_CR1_DNF; // disable dig filters
	I2C1->CR1 |= I2C_CR1_ERRIE; // allow error interrupts
	I2C1->CR1 &= ~ I2C_CR1_NOSTRETCH; // allow clock stretching

	I2C1->CR1 |= I2C_CR1_ADDRIE; // enable slave address match interrupt

	// Timing config
	I2C1->TIMINGR = 0; // clear timing reg
	I2C1->TIMINGR &= ~I2C_TIMINGR_PRESC; //clear presc
	I2C1->TIMINGR |= 9<<28; // set presc to 9 (now clock is 400 kHz)
	I2C1->TIMINGR |= 32; // set SCLL to 5
	I2C1->TIMINGR |= 26<<8; // set SCLH to 5
	I2C1->TIMINGR |= 0x2<<16; // set SDADEL to table value
	I2C1->TIMINGR |= 0x4<<20; // set scaldel to table value

	// set i2c own address register
	I2C1->OAR1 &= ~I2C_OAR1_OA1EN;
	I2C1->OAR1 |= I2C_OAR1_OA1EN|(slaveOwnAddr<<1); // set the own address
	I2C1->OAR2 &=~I2C_OAR2_OA2EN; // disable oar 2

	// i2c cr2 config
	I2C1->CR2 &= ~ I2C_CR2_ADD10; // do 7 bit addressing
	I2C1->CR2 |= I2C_CR2_AUTOEND; // en autoend
	I2C1->CR1 |= I2C_CR1_PE; // enable peripheral
}


void I2C2GPIOINIT(void){
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN; // enable clock to gpiob
	GPIOB->MODER &= ~(GPIO_MODER_MODE12 | GPIO_MODER_MODE11 | GPIO_MODER_MODE10); // clear mode regs
	GPIOB->MODER |= GPIO_MODER_MODE12_1 | GPIO_MODER_MODE11_1 | GPIO_MODER_MODE10_1; // make them alternate functions
	GPIOB->OTYPER |= 7<<10; // make open drain pins
	GPIOB->AFR[1] &= ~(0xFFF<<8);
	GPIOB->AFR[1] |= 4<<8; // set to af4 (pb10 is I2C SCL)
	GPIOB->AFR[1] |= 4<<12;// set to af4 (pb11 is I2C SDA)
	GPIOB->AFR[1] |= 4<<16;// set to af4 (pb12 is I2C SMBA)
}// end i2c2gpioinit

void I2C1GPIOINIT(void){
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
	I2C->CR2 |= I2C_CR2_START; // send start bit
	// PROCESS: sends start bit, starts clock, sends 7 bit address, holds low for for write/goes high for read,
	// waits for slave ack (pull down line after w/r bit), and then low
}

/*
int I2C_getCellVoltaddr(char cellnum) // get address for reading cell voltage
{   // cellnum: 0, 1 , 2, 3, 4, 5, b
	// returns the concatenated hi addr and lo addr
	int addr;
	switch(cellnum) {
		case '1': // address for VC1 lo and hi
			addr = (0x0C<<2) + 0x0D; // high is shifted, low is not
			break;
		case '2': // VC2
			addr = (0x0E<<2) + 0x0F;
			break;
		case '3':
			addr = (0x10<<2) + 0x11;
			break;
		case '4':
			addr = (0x12<<2) + 0x13;
			break;
		case '5':
			addr = (0x14<<2) + 0x15;
			break;
		case 'b':
			addr = (0x2A<<2) + 0x2B;
			break;
		default:
			addr = 0xFFFF; // erroraddr
	}
	return addr;
} // end I2C_getCellVoltaddr */

void I2CreadfromBatt(int addr) { // todo: add to header file
	// This function configures i2c to read from the battery monitor
	// read reg is register name to read
	uint32_t bqAddr = 0x08;
	int Size = 1; // read one byte from register TODO: should siz be data size + 1 for the reg address
	I2C_StartTX(I2C2,  bqAddr, Size, MASTERREAD);
}
/* USER CODE END 0 */
