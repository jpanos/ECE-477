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
	    16.7412175,
	    16.69617777,
      16.6516577,
      16.6077044,
      16.56436256,
      16.52167451,
      16.47968017,
      16.43841709,
      16.39792041,
      16.35822291,
      16.31935496,
      16.28134455,
      16.24421727,
      16.20799635,
      16.1727026,
      16.13835446,
      16.10496798,
      16.07255682,
      16.04113224,
      16.01070314,
      15.981276,
      15.95285493,
      15.92544165,
      15.89903549,
      15.8736334,
      15.84922992,
      15.82581722,
      15.80338507,
      15.78192088,
      15.76140963,
      15.74183395,
      15.72317405,
      15.70540777,
      15.68851056,
      15.67245548,
      15.6572132,
      15.642752,
      15.62903778,
      15.61603405,
      15.60370192,
      15.59200013,
      15.58088502,
      15.57031054,
      15.56022825,
      15.55058734,
      15.5413346,
      15.53241442,
      15.52376883,
      15.51533744,
      15.50705749,
      15.49886383,
      15.49068892,
      15.48246283,
      15.47411325,
      15.46556547,
      15.4567424,
      15.44756456,
      15.43795008,
      15.4278147,
      15.41707178,
      15.40563229,
      15.3934048,
      15.3802955,
      15.36620819,
      15.35104429,
      15.33470283,
      15.31708044,
      15.29807137,
      15.27756748,
      15.25545824,
      15.23163074,
      15.20596968,
      15.17835736,
      15.1486737,
      15.11679624,
      15.08260011,
      15.04595808,
      15.00674051,
      14.96481537,
      14.92004827,
      14.87230239,
      14.82143857,
      14.76731521,
      14.70978837,
      14.64871169,
      14.58393642,
      14.51531146,
      14.44268327,
      14.36589596,
      14.28479123,
      14.19920841,
      14.10898442,
      14.01395382,
      13.91394876,
      13.808799,
      13.69833192,
      13.58237252,
      13.4607434,
      13.33326478,
      13.19975447
	};

void sendNano(void){
	// send charachters to nano
	Txdata txdata;
	txdata.battPercent = shared->batPercentRemain;
	txdata.voltage = shared->voltage;
	char * structaddr = (char *) &txdata;
	int size = sizeof(txdata);
	for (int k = 0;  k < size; k++){
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
	shared->usartct = 0;
	shared->flowercoord.x = 0;
	shared->flowercoord.y = 0;
	shared->flowercoord.z = 0;
	//set gpio
	GPIOC->MODER &= ~(GPIO_MODER_MODE10|GPIO_MODER_MODE11);
	GPIOC->MODER |= GPIO_MODER_MODE10_1 | GPIO_MODER_MODE11_1; // make alternate functions
	GPIOC->OTYPER &= ~(0x3<<10); // make push pull
	GPIOC->AFR[1] &= ~(0xFF<<8);
	GPIOC->AFR[1] |= (7<<8) | (7<<12); // set to AF7.

	// config uart
	USART3->BRR = 64000000 / 115200; // set the baudrate
	USART3->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);
	USART3->CR2 &= ~(USART_CR2_STOP); // set 1 stop bit
	USART3->CR1 |= USART_CR1_RXNEIE; // enable interrupt for rx not empty
	USART3->CR2 |= USART_CR2_RTOEN;
	USART3->RTOR = 5;
	USART3->CR1 &= ~USART_CR1_RTOIE; // enable interrupt for rx timeout
	NVIC_EnableIRQ(USART3_IRQn); // enable interrupt
	USART3->CR1 |= USART_CR1_UE | USART_CR1_RE | USART_CR1_TE; // enable recieve and enable peripheral
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
