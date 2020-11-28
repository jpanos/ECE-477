#include "i2c_battery.h"
	extern char masterrxdata;
	extern char mastertxdata;
	extern int i2cTargReg;
	extern char i2cmode;

	typedef struct _txdata {
			int battPercent;
			float voltage;
		}Txdata;

	float batValLUT[100] =
	{
			16.40705651,
			16.0901087,
			15.80801,
			15.55817722,
			15.33814352,
			15.14555591,
			14.97817283,
			14.83386164,
			14.71059618,
			14.6064543,
			14.51961541,
			14.44835799,
			14.39105712,
			14.34618205,
			14.31229373,
			14.2880423,
			14.27216469,
			14.26348209,
			14.26089754,
			14.26339345,
			14.2700291,
			14.27993824,
			14.29232657,
			14.30646928,
			14.32170863,
			14.33745144,
			14.35316664,
			14.36838281,
			14.38268572,
			14.39571583,
			14.40716587,
			14.41677837,
			14.42434316,
			14.42969494,
			14.43271079,
			14.43330774,
			14.43144027,
			14.42709786,
			14.42030252,
			14.41110633,
			14.399589,
			14.38585535,
			14.37003289,
			14.35226934,
			14.33273015,
			14.31159608,
			14.28906068,
			14.26532787,
			14.24060946,
			14.21512267,
			14.18908768,
			14.16272517,
			14.13625384,
			14.10988796,
			14.08383491,
			14.05829268,
			14.03344745,
			14.0094711,
			13.98651875,
			13.96472629,
			13.94420793,
			13.92505373,
			13.90732711,
			13.89106244,
			13.87626251,
			13.86289612,
			13.85089559,
			13.84015428,
			13.83052417,
			13.82181335,
			13.81378359,
			13.80614785,
			13.79856782,
			13.79065146,
			13.78195056,
			13.77195822,
			13.76010643,
			13.74576359,
			13.72823205,
			13.70674564,
			13.68046721,
			13.64848615,
			13.60981595,
			13.56339173,
			13.50806775,
			13.44261498,
			13.36571861,
			13.2759756,
			13.1718922,
			13.05188151,
			12.91426099,
			12.75725001,
			12.57896738,
			12.37742889,
			12.15054484,
			11.89611757,
			11.611839,
			11.29528819,
			10.94392884,
			10.55510682
	};

void sendNano(void){
	// send charachters to nano
	Txdata txdata;
	txdata.battPercent = shared->batPercentRemain;
	txdata.voltage = shared->voltage;
	char * structaddr = (char *) &txdata;
	for (int k = 0;  k < sizeof(txdata); k++){
		while((USART3->ISR & USART_ISR_TC) != USART_ISR_TC){};
		USART3->TDR = structaddr[k];
	}
}


void getBatPercent(void) {
	int k = 1;
	float voltage = shared->voltage;
	while (k < 100){
		if (voltage > batValLUT[k]){
			shared->batPercentRemain = 100-k;
			k = 101;
		}
		k++;
	}
	if (k == 100)
		shared->batPercentRemain = 0;
}

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
	int data = shared->masterrxdata;
	// vc1
	if (reg == (0x0c)) {// save c
		shared->VC1 = shared->masterrxdata << 8;
		shared->VC1 &= ~(0x3<<14);
	}
	else if (reg == (0x0d))
		shared->VC1 |= shared->masterrxdata;
	//vc2
	else if (reg == (0x0e)){
		shared->VC2 = shared->masterrxdata <<8;
		shared->VC2 &= ~(0x3<<14);
	}
	else if (reg ==(0x0f))
		shared->VC2 |= shared->masterrxdata;
	//vc3
	else if (reg == (0x10)){
		shared->VC3 = shared->masterrxdata <<8;
		shared->VC3 &= ~(0x3<<14);
	}
	else if (reg == (0x11))
		shared->VC3 |= shared-> masterrxdata;
	//vc4
	else if (reg == (0x12)){
		shared->VC4 = shared->masterrxdata <<8;
		shared->VC4 &= ~(0x3<<14);
	}
	else if (reg == (0x13))
		shared->VC4 |= shared->masterrxdata;
	//vc5
	else if (reg == (0x14)){
		shared->VC5 = shared->masterrxdata<<8;
		shared->VC5 &= ~(0x3<<14);
	}
	else if (reg == (0x15))// save 0x15
		shared->VC5 |= shared->masterrxdata;
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
//			size++; //currently reading 1 byte
			I2C_StartTX(I2C2, bqaddr, size, MASTERREAD);
		}
	}
}

void initI2C2(void){
	// This function initializes I2C
	uint32_t OwnAddr = 0x52; // decide own address
	shared->regReading = 0x0b;
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


void initUART(void){
	//pc10 is uart3tx
	//pc11 is uart3rx
	// AF 7
	// 115200 BAUDRATE
	RCC->APB1LENR |= RCC_APB1LENR_USART3EN; // enable clock
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOCEN;

	//set gpio
	GPIOC->MODER &= ~(GPIO_MODER_MODE10|GPIO_MODER_MODE11);
	GPIOC->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1; // make alternate functions
	GPIOC->OTYPER &= ~(0x3<<10); // make push pull
	GPIOC->AFR[1] &= ~(0xFF<<8);
	GPIOC->AFR[1] |= (7<<8) | (7<<12); // set to AF7.

	// config uart
	USART3->BRR = 115200; // set the baudrate
	USART3->CR1 |= USART_CR1_UE | USART_CR1_TE; // enable transmit and enable peripheral
}

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
