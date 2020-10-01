#include <mavlink_pssc.h>

mavlink_system_t mavlink_system = {
		1, // System ID (0-255)
		MAV_COMP_ID_ONBOARD_COMPUTER // Computer ID (a MAV_COMPONENT value)
};

mavlink_message_t _hb_msg;
__attribute__((section(".dma_buffer"))) uint8_t msg_buff[263];

uint8_t _initialize_UART_DMA(void) {
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN;
	GPIOB->MODER &= ~(GPIO_MODER_MODE14);
	GPIOB->MODER |= GPIO_MODER_MODE14_0;

	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
	GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2);
	GPIOA->AFR[1] |= (7 << 4) | (7 << 8);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1); //1 start bit, 8 data bits, n stob bits
	USART1->BRR = 64000000 / 57600; // fixed?
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE; // enable rx and tx
	USART1->CR2 &= ~(USART_CR2_STOP); // 1 stop bit
	USART1->CR3 |= USART_CR3_DMAT;
	USART1->CR1 |= USART_CR1_UE;

	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	DMA1_Stream0->CR |= DMA_MINC_ENABLE | DMA_SxCR_DIR_0 | DMA_SxCR_TRBUFF;
	DMA1_Stream0->PAR = (unsigned int) &USART1->TDR;

	DMAMUX1_Channel0->CCR = 42; // uart1_tx_req42
	// DMA1_Stream0->CR |= DMA_SxCR_EN;

	RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
	TIM6->PSC = 65000 - 1;
	TIM6->ARR = 1000;
	TIM6->DIER |= TIM_DIER_UIE;

	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->CR1 |= TIM_CR1_CEN;

	return 0;
}

uint8_t _heartbeat_msg_initialize(void) {
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, 			 // system_id
                             mavlink_system.compid,			 // component_id
                             &_hb_msg,                  		 // mavlink_message_t
                             MAV_TYPE_QUADROTOR,   			 // MAV_TYPE
                             MAV_AUTOPILOT_PX4,    			 // MAV_AUTOPILOT
                             MAV_MODE_FLAG_AUTO_ENABLED, // base_mode
                             0,                    			 // custom_mode
                             MAV_STATE_ACTIVE      			 // system_status
                             );
  return 0;
}

uint8_t mavlink_initialize(void) {
	_initialize_UART_DMA();
	_heartbeat_msg_initialize();
	return 0;
}

uint8_t send_mavlink_message(mavlink_message_t *msg) {
	while ((DMA1_Stream0->CR & 0x1) == 1) {}
	DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CTEIF0;
	uint8_t * pointer = msg_buff;
	mavlink_msg_to_send_buffer(msg_buff, msg);
	DMA1_Stream0->NDTR = msg->len + 12;
	DMA1_Stream0->M0AR = (unsigned int) msg_buff;
  DMA1_Stream0->CR |= DMA_SxCR_EN;
  return 0;
}

void TIM6_DAC_IRQHandler() {
	TIM6->SR &= ~TIM_SR_UIF; // Need to put clear flag up hear, or at least before on other instruction (not directly by bracket)
	// otherwise NVIC will not register and IRQHandler will fire again.

	GPIOB->ODR ^= GPIO_ODR_OD14;
	send_mavlink_message(&_hb_msg);
	// DMA1_Stream0->NDTR = 12;

	//TIM6->CR1 &= TIM_CR1_CEN;
}
