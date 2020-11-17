#include "i2c_battery.h"
	extern char masterrxdata;
	extern char mastertxdata;
	extern int i2cTargReg;
	extern char i2cmode;

void computeVoltages(void){
	shared->cell1 = shared->VC1 * 6.275 / (16384.0-1);
	shared->cell2 = (shared->VC2-shared->VC1)  * 6.275 / (16384.0 -1);
	shared->cell3 = (shared->VC3-shared->VC2)  * 6.275 / (16384.0 -1);
	shared->cell4 = (shared->VC5-shared->VC4)  * 6.275 / (16384.0 -1);
	shared->total = (shared->bat) * 6.275 / (16384.0-1);
	// todo unmask handler here
}

void storeVData(void) {
	int reg = shared->regReading;
	if (reg == (0x0c+1)) // save c
		shared->VC1 = shared->masterrxdata << 8;
	else if (reg == (0x0d+1))
		shared->VC1 += shared->masterrxdata;
	else if (reg == (1+0x0e))
		shared->VC2 = shared->masterrxdata <<8;
	else if (reg ==(0x0f+1))
		shared->VC2 += shared->masterrxdata;
	else if (reg == (0x10+1))
		shared->VC3 = shared->masterrxdata <<8;
	else if (reg == (0x11+1))
		shared->VC3 += shared-> masterrxdata;
	else if (reg == (0x12+1))
		shared->VC4 = shared->masterrxdata <<8;
	else if (reg == (0x13+1))
		shared->VC4 += shared->masterrxdata;
	else if (reg == (0x14+1))
		shared->VC5 = shared->masterrxdata<<8;
	else if (reg == (0x2a))// save 0x15
		shared->VC5 += shared->masterrxdata;
	else if (reg == (0x2a+1))
		shared->bat = shared->masterrxdata<<8;
	else if (reg == (0xc)) // save 0x2b
		shared->bat += shared->masterrxdata;
}

void I2C2battTalk(int writeMode, int regAddr, char byte){
	// this function performs a read or write sequence to the bq76920
	// right now enables 1 byte writing
	// inputs: MASTERREAD/MASTERWRITE, target register address, byte to write
	int bqaddr = 0x08;
	shared->i2cTargReg = regAddr;
	shared->i2cmode = writeMode;
	if (writeMode == MASTERWRITE)
	{
		shared->mastertxdata = byte; // byte to send
		int size = 1; // send two bytes: reg and data
		I2C_StartTX(I2C2, bqaddr, size, MASTERWRITE); // send the slave address and write request
		I2C2->CR1 |= I2C_CR1_TXIE; // allow transmitter empty interrupt
	}
	else // do read
	{
		int size = 1;
		if (shared->i2cTargReg != -1) {
			I2C_StartTX(I2C2, bqaddr, size, MASTERWRITE);
			I2C2->CR1 |= I2C_CR1_TXIE; // allow tx emply interrupt so can send dest reg.
		}
		else{
//			size+; currently reading 1 byte
			I2C_StartTX(I2C2, bqaddr, size, MASTERREAD);
		}
	}
}

void initI2C2(void){
	// This function initializes I2C
	uint32_t OwnAddr = 0x52; // decide own address
	shared->regReading = 0x0c;
	shared->computeVoltageFlag = 0;
	shared->VC1 = 0;
	shared->VC2 = 0;
	shared->VC3 = 0;
	shared->VC4 = 0;
	shared->VC5 = 0;
	shared->bat = 0;
	shared->cell1 = 0;
	shared->cell2 = 0;
	shared->cell3 = 0;
	shared->cell4 = 0;
	NVIC_EnableIRQ(I2C2_EV_IRQn);
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
	I2C2->TIMINGR |= 14<<28; // set presc to 9 (now clock is 400 kHz)
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

void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction) {
	I2C->CR2 &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD|I2C_CR2_RD_WRN|I2C_CR2_ADD10);
	if (Direction) // requests a read transfer
	{
		I2C->CR2 |= I2C_CR2_RD_WRN;
	}
	I2C->CR2 |= DevAddress <<1;
	I2C->CR2 &= ~((0xFF)<<16);
	I2C->CR2 |= Size<<16;
	I2C->CR2 |= I2C_CR2_START; // send start bit
	// PROCESS: sends start bit, starts clock, sends 7 bit address, holds low for for write/goes high for read,
	// waits for slave ack (pull down line after w/r bit), and then low
}
