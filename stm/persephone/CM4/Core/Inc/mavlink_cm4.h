/*
 * mavlink_cm4.h
 *
 *  Created on: Oct 9, 2020
 *      Author: moiz
 */

#ifndef INC_MAVLINK_CM4_H_
#define INC_MAVLINK_CM4_H_

#include <stm32h7xx.h>
#include <mavlink_pssc.h>

// initializes all IO and starts sending the
// heartbeat message at a rate of 1Hz
uint8_t mavlink_initialize(void);

#endif /* INC_MAVLINK_CM4_H_ */
