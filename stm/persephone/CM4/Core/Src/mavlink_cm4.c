/*
 * mavlink_cm4.c
 *
 *  Created on: Oct 9, 2020
 *      Author: moiz
 */

#include <mavlink_cm4.h>
#include "i2c_battery.h"

// receive structs
mavlink_message_t _rcv_msg;
mavlink_status_t _rcv_msg_stat;
mavlink_heartbeat_t _rcv_msg_heartbeat;
mavlink_trajectory_representation_waypoints_t _rcv_msg_traj_way;
mavlink_global_position_int_t _rcv_msg_gps_int;
mavlink_gps_raw_int_t _rcv_msg_gps_raw_int;
mavlink_altitude_t _rcv_msg_altitude;
mavlink_local_position_ned_t _rcv_local_pos_ned;
mavlink_ping_t _rcv_msg_ping;
mavlink_extended_sys_state_t _rcv_extended_sys_state;
mavlink_command_int_t _rcv_cmd_int;
mavlink_command_long_t _rcv_cmd_long;
mavlink_command_ack_t _rcv_cmd_ack;

// transmit structs
mavlink_message_t _hb_msg;

uint8_t _heartbeat_msg_initialize(void) {
  mavlink_msg_heartbeat_pack(mavlink_system.sysid, 			 // system_id
                             mavlink_system.compid,			 // component_id
                             &_hb_msg,                   // mavlink_message_t
                             MAV_TYPE_ONBOARD_CONTROLLER,// MAV_TYPE
                             MAV_AUTOPILOT_INVALID,			 // MAV_AUTOPILOT
                             MAV_MODE_FLAG_AUTO_ENABLED, // base_mode
                             0,                    			 // custom_mode
                             MAV_STATE_ACTIVE      			 // system_status
                             );
  return 0;
}

uint8_t _initialize_UART_DMA(void) {
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN;
	// put red led in output mode
	GPIOB->MODER &= ~(GPIO_MODER_MODE14);
	GPIOB->MODER |= GPIO_MODER_MODE14_0;
	GPIOB->ODR ^= GPIO_ODR_OD14;

	// UART configuration
	// configure tx and rx pins of USART1
	GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
	GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;
	GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1 | GPIO_AFRH_AFRH2);
	GPIOA->AFR[1] |= (7 << 4) | (7 << 8);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	USART1->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1); //1 start bit, 8 data bits, n stob bits
	USART1->BRR = 64000000 / 57600; // set baudrate to 57600
	USART1->CR1 |= USART_CR1_RE | USART_CR1_TE; // enable rx and tx
	USART1->CR2 &= ~(USART_CR2_STOP); // 1 stop bit
	USART1->CR3 |= USART_CR3_DMAT; // enable transmit DMA

	USART1->CR1 |= USART_CR1_RXNEIE; // recieve not empty interrupt enable
	NVIC_EnableIRQ(USART1_IRQn);

	USART1->CR1 |= USART_CR1_UE;

	// DMA configuration
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	// increment memory location, memory to peripheral, said to enable for usart idk
	DMA1_Stream0->CR |= DMA_MINC_ENABLE | DMA_SxCR_DIR_0 | DMA_SxCR_TRBUFF;
	DMA1_Stream0->PAR = (unsigned int) &USART1->TDR;
	// transfer complete interrupt enable
	DMA1_Stream0->CR |= DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	DMAMUX1_Channel0->CCR = 42; // uart1_tx_req42


	return 0;
}

uint8_t _initialize_timers() {

	// enable timer interrupt to go off 20Hz
	// configured if main clock is 64 MHz
	// for heartbeat message

	RCC->APB1LENR |= RCC_APB1LENR_TIM6EN;
	TIM6->PSC = 64000 - 1;
	TIM6->ARR = 10;
	TIM6->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	TIM6->CR1 |= TIM_CR1_CEN;

	return MVPSSC_SUCCESS;
}

uint8_t _initialize_queue(Queue * q, ListNode * ll_buff, void * data_array,
														int length, int data_element_size) {
	char * temp = (char *) data_array;
	init_queue(q);
	for (int i = 0; i < length; i++) {
		ll_buff[i].next = NULL;
		enqueue(q, &ll_buff[i]);
		ll_buff[i].data = &temp[data_element_size * i];
	}
	return 0;
}

uint8_t mavlink_initialize(void) {
	spin_lock(HSEM_ID_EMPTY, 0);
	_initialize_queue(&shared->empty_nodes, &shared->nodes, &shared->msgs_data,
											MVPSSC_MSG_BUFF_SIZE, sizeof(MsgData));
	lock_release(HSEM_ID_EMPTY, 0);

	spin_lock(HSEM_ID_MSGS, 0);
	init_queue(&shared->send_msgs);
	lock_release(HSEM_ID_MSGS, 0);

	shared->cmd.node = NULL;
	shared->latitude = -1;
	shared->longitude = -1;

	shared->mav_state = 0;
	shared->mav_mode_flag = 0;
	// set position mask to ignore all
	// shared->pos_type_mask = MVPSSC_POS_MASK_IGNORE_ALL;
	shared->pos_type_mask = MVPSSC_POS_MASK_TAKEOFF;
	shared->pos_mode |= MVPSSC_POS_MODE_EN;
	set_pos_freq(4);

	_initialize_UART_DMA();
	_heartbeat_msg_initialize();
	_initialize_timers();
	return 0;
}

uint8_t send_next_msg() {
	if ((DMA1_Stream0->CR & DMA_SxCR_EN) == DMA_SxCR_EN) {
		return MVPSSC_FAIL;
	}
	// clear flags that might appear from previous DMA transfer, won't enable otherwise
	DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CTEIF0;

	ListNode * n;
	n = shared->send_msgs.head;

	if (n == NULL) return MVPSSC_FAIL;

	MsgData * msgdata = (MsgData *) n->data;

	// set dma transfer length, +12 is for header
	DMA1_Stream0->NDTR = msgdata->len;
	// set DMA location
	DMA1_Stream0->M0AR = (unsigned int) msgdata->buff;
	// enable DMA
  DMA1_Stream0->CR |= DMA_SxCR_EN;
  return MVPSSC_SUCCESS;
}

uint8_t _send_position_target() {
	mavlink_message_t msg;
	spin_lock(HSEM_ID_POS_SETPOINT, TIM6_PROC_ID);
	mavlink_msg_set_position_target_local_ned_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			shared->time_boot_ms,
			mavlink_autopilot.sysid,
			mavlink_autopilot.compid,
			shared->pos_coord_frame,
			shared->pos_type_mask,
			shared->pos_set_x,
			shared->pos_set_y,
			shared->pos_set_z,
			shared->pos_set_vx,
			shared->pos_set_vy,
			shared->pos_set_vz,
			shared->pos_set_afx,
			shared->pos_set_afy,
			shared->pos_set_afz,
			shared->pos_set_yaw,
			shared->pos_set_yaw_rate);
	lock_release(HSEM_ID_POS_SETPOINT, TIM6_PROC_ID);
	if (queue_message(&msg, TIM6_PROC_ID) != NULL) return MVPSSC_SUCCESS;
	else return MVPSSC_FAIL;
}

int msg_buff_vals[30];
void put_val_in_buff(uint32_t val) {
	for (int i = 0; i < 30; i++) {
		if (msg_buff_vals[i] == val) break;
		else if (msg_buff_vals[i] != 0) continue;
		else {
			msg_buff_vals[i] = val;
			break;
		}
	}
}

void _cmd_cleanup() {
		ListNode * n = shared->cmd.node;
		shared->cmd.node = NULL;

		spin_lock(HSEM_ID_EMPTY, UART1_RX_PROC_ID);
		enqueue(&shared->empty_nodes, n);
		lock_release(HSEM_ID_EMPTY, UART1_RX_PROC_ID);

		shared->cmd.done = MVPSSC_CMD_DONE;
}

void _handle_cmd_ack() {
	shared->cmd.result = _rcv_cmd_ack.result;
	if (shared->cmd.cmd_id == MAV_CMD_NAV_TAKEOFF) {
		int i = 0;
	}

	switch (_rcv_cmd_ack.result) {
		case MAV_RESULT_ACCEPTED:
		case MAV_RESULT_TEMPORARILY_REJECTED:
		case MAV_RESULT_DENIED:
		case MAV_RESULT_UNSUPPORTED:
		case MAV_RESULT_FAILED:
		case MAV_RESULT_IN_PROGRESS:
		case MAV_RESULT_CANCELLED:
			_cmd_cleanup();
			break;
	}
}

uint8_t parse_mavlink_message(mavlink_message_t *msg) {
	put_val_in_buff(msg->msgid);
	switch (msg->msgid) {
		case MAVLINK_MSG_ID_HEARTBEAT:
			mavlink_msg_heartbeat_decode(msg, &_rcv_msg_heartbeat);
			shared->mav_state = _rcv_msg_heartbeat.system_status;
			shared->mav_mode_flag = _rcv_msg_heartbeat.base_mode;
			break;
		case MAVLINK_MSG_ID_PING:
			mavlink_msg_ping_decode(msg, &_rcv_msg_ping);
			break;
		case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
			mavlink_msg_extended_sys_state_decode(msg, &_rcv_extended_sys_state);
			shared->landed_state = _rcv_extended_sys_state.landed_state;
			break;
		case MAVLINK_MSG_ID_ALTITUDE:
			mavlink_msg_altitude_decode(msg, &_rcv_msg_altitude);
			shared->altitude_rel = _rcv_msg_altitude.altitude_relative;
			shared->altitude_msl = _rcv_msg_altitude.altitude_amsl;
			break;
		case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			mavlink_msg_global_position_int_decode(msg, &_rcv_msg_gps_int);
			shared->longitude = _rcv_msg_gps_int.lon;
			shared->latitude = _rcv_msg_gps_int.lat;
			break;
		case MAVLINK_MSG_ID_GPS_RAW_INT:
			mavlink_msg_gps_raw_int_decode(msg, &_rcv_msg_gps_raw_int);
			shared->longitude_raw = _rcv_msg_gps_raw_int.lon;
			shared->latitude_raw = _rcv_msg_gps_raw_int.lat;
			break;
		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			mavlink_msg_local_position_ned_decode(msg, &_rcv_local_pos_ned);
			shared->pos_x = _rcv_local_pos_ned.x;
			shared->pos_y = _rcv_local_pos_ned.y;
			shared->pos_z = _rcv_local_pos_ned.z;
			break;
		case MAVLINK_MSG_ID_COMMAND_INT:
			mavlink_msg_command_int_decode(msg, &_rcv_cmd_int);
			break;
		case MAVLINK_MSG_ID_COMMAND_LONG:
			mavlink_msg_command_long_decode(msg, &_rcv_cmd_long);
			break;
		case MAVLINK_MSG_ID_COMMAND_ACK:
			mavlink_msg_command_ack_decode(msg, &_rcv_cmd_ack);
			_handle_cmd_ack();
			break;
	}
	return 0;
}

uint8_t _tim6_hb_fire_count = 0;
int _tim6_hb_secs = 0;
void TIM6_DAC_IRQHandler() {
	TIM6->SR &= ~TIM_SR_UIF; // Need to put clear flag up hear, or at least
	// before on other instruction (not directly by bracket)
	// otherwise NVIC will not register and IRQHandler will fire again.

	// increment sys time by 50 ms
	shared->time_boot_ms += 10;

	// blink light
	GPIOB->ODR ^= GPIO_ODR_OD14;
	// send heartbeat message at 1Hz
	if (shared->time_boot_ms % 500 == 0) {
		queue_message(&_hb_msg, TIM6_PROC_ID);
	}

	// send positional update at 4Hz
	if ((shared->pos_mode & MVPSSC_POS_MODE_EN) && (shared->time_boot_ms % shared->pos_period == 0)) {
		_send_position_target();
	}

	if (shared->time_boot_ms % 50 == 0 && ((DMA1_Stream0->CR & 0x1) != 1)) {
		send_next_msg();
	}
	if (shared->time_boot_ms % 1000 == 0 && ~(shared->computeVoltageFlag)){
		// store prev cycle data here
		shared->count = 8;
		shared->voltage = (shared->bat) * 6.275 / (16383);
		int reg = 0x2a;
//		storeVData();
		char send = 0x18;// byte to send
		I2C2battTalk(MASTERREAD,reg,send);
//		reg++;
//		if (reg == 0x16)
//			reg = 0x2a;
//		if (reg == 0x2c) {
//			reg = 0xc;
//			shared->computeVoltageFlag = 1; // set compute voltage flag
//		}
//		shared->regReading = reg;
	}
//	if (shared->computeVoltageFlag) {
//		storeVData();
//		shared->computeVoltageFlag = 0; // clear the flag
//		computeVoltages();
//	}

}

void USART1_IRQHandler() {
	// check if handler was fired because of receive buffer not empty flag
	if ((USART1->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE) {
		// mavlink helper function parses message by passing it one char at a time
		// returns 1 when successfully received/parsed 1 mavlink message
		// reading from RDR clears RXFNE flag
		if (mavlink_parse_char(MAVLINK_COMM_0, USART1->RDR, &_rcv_msg, &_rcv_msg_stat)) {
			parse_mavlink_message(&_rcv_msg);
		}
	}
	USART1->ICR |= 0x123bff;
}

void DMA1_Stream0_IRQHandler() {
	if ((DMA1->LISR & DMA_LISR_TCIF0) == DMA_LISR_TCIF0) {
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
		ListNode * n;
		spin_lock(HSEM_ID_MSGS, DMA1_S0_PROC_ID);
		n = dequeue(&shared->send_msgs);
		lock_release(HSEM_ID_MSGS, DMA1_S0_PROC_ID);

		MsgData * msgdata = (MsgData *) n->data;

		if (shared->cmd.node != n) {
			spin_lock(HSEM_ID_EMPTY, DMA1_S0_PROC_ID);
			enqueue(&shared->empty_nodes, n);
			lock_release(HSEM_ID_EMPTY, DMA1_S0_PROC_ID);
		}

		send_next_msg();
	}
	// DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CFEIF0 | DMA_LIFCR_CTEIF0;
}
