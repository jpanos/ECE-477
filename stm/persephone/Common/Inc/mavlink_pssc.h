/*
 * mavlink_pscc.h
 *
 *  Created on: Sep 28, 2020
 *      Author: moiz
 *
 *  Description: mavlink implementation:
 *  Requirments: USART1, TIM6, DAC1_Stream0
 */

#ifndef INC_MAVLINK_PSSC_H_
#define INC_MAVLINK_PSSC_H_

#include <stm32h7xx_hal.h>
#include <common/mavlink.h>

// initializes all IO and starts sending the
// heartbeat message at a rate of 1Hz
uint8_t mavlink_initialize(void);

// will send a message to flight controller instructing it to send
// the message_id specified at the interval specified
uint8_t set_mavlink_msg_interval(uint16_t message_id, int32_t interval_us);


#endif /* INC_MAVLINK_PSSC_H_ */
