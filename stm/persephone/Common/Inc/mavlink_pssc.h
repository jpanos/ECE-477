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
#include <spin_lock.h>
#include <Queue.h>

#define MVPSSC_SUCCESS 0
#define MVPSSC_FAIL 1

#define MVPSSC_MSG_BUFF_SIZE 5
#define MVPSSC_MSG_MAX_SIZE 263
#define MVPSSC_SHARED_SRAM_LOC 0x30040000

#define HSEM_ID_EMPTY 15
#define HSEM_ID_MSGS 16
#define HSEM_ID_CMD 17
#define HSEM_ID_CMD_BLOCK HSEM_ID_CMD

#define DMA1_S0_PROC_ID 40
#define TIM6_PROC_ID 41
#define UART1_RX_PROC_ID 42
#define CMD_BLOCK_PROC_ID 43

#define MVPSSC_IS_CMD 1
#define MVPSSC_IS_NOT_CMD 0
#define MVPSSC_CMD_DONE 1
#define MVPSSC_CMD_START 0

extern mavlink_system_t mavlink_system;

typedef struct _msg_data {
	uint8_t buff[MVPSSC_MSG_MAX_SIZE];
	uint8_t len;
	uint8_t is_cmd;
} MsgData;

typedef struct _cmd_data {
	ListNode * node;
	uint32_t cmd_id;
	uint8_t result;
	uint8_t done;
} CmdData;

typedef struct _mavlink_shared_data {
	ListNode nodes[MVPSSC_MSG_BUFF_SIZE];
	MsgData msgs_data[MVPSSC_MSG_BUFF_SIZE];
	Queue empty_nodes;
	Queue send_msgs;
	CmdData cmd;

	float altitude_rel;
	float altitude_msl;
	int latitude_raw;
	int longitude_raw;
	int latitude;
	int longitude;
} MavlinkSharedData;

extern volatile MavlinkSharedData * const mv_shared;

ListNode * queue_message(mavlink_message_t * msg, uint32_t procID);

// IMPORTANT: only meant to be called by main process in CM7
uint8_t send_command(uint32_t cmdid, mavlink_message_t * msg);

// will send a message to flight controller instructing it to send
// the message_id specified at the interval specified
uint8_t set_mavlink_msg_interval(uint16_t message_id, int32_t interval_us);

// send ping message
uint8_t send_ping_message(void);

// arm/disarm
// armval: 1 to arm, 0 to disarm
// force: 1 to force state, 0 to not
uint8_t send_arm_disarm_message(uint8_t armval, uint8_t force);

#endif /* INC_MAVLINK_PSSC_H_ */
