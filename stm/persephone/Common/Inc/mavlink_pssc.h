/*
 * mavlink_pscc.h
 *
 *  Created on: Sep 28, 2020
 *      Author: moiz
 *
 *  Description: mavlink implementation:
 *  Requirments: USART1, TIM6, DAC1_Stream0
 */

/*
 * TODO:
 *   - global position setpoint
 *   - dynamically change setpoint
 *   - change bit masks
 *
 *   - send heartbeat more frequently
 *   - check logs all the time
 */

#ifndef INC_MAVLINK_PSSC_H_
#define INC_MAVLINK_PSSC_H_

#define MVPSSC_SUCCESS 0
#define MVPSSC_FAIL 1

#define MVPSSC_MSG_BUFF_SIZE 5
#define MVPSSC_MSG_MAX_SIZE 263

#define MVPSSC_IS_CMD 1
#define MVPSSC_IS_NOT_CMD 0
#define MVPSSC_CMD_DONE 1
#define MVPSSC_CMD_START 0

#define MVPSCC_OFFBOARD_START 1
#define	MVPSSC_OFFBOARD_STOP 0

#define MVPSSC_POS_MODE_EN 0x1
#define MVPSSC_POS_MODE_FLOWER 0x2
#define MVPSSC_POS_MODE_LAND 0x80

// mav mask reference: yaw_rate yaw force afz | afy afx vz vy | vx z y x
#define MVPSSC_POS_MASK_IGNORE_ALL 0x0dff
#define MVPSSC_POS_MASK_POSITION_SETPOINT 0x0df8
#define MVPSSC_POS_MASK_VELOCITY_SETPOINT 0x0dc7
#define MVPSSC_POS_MASK_VEL_YAWRATE_SETPOINT 0x05c7


#define PX4_MODE (MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED)
#define PX4_CUSTOM_MODE_OFFBOARD 6
#define PX4_CUSTOM_MODE_AUTO 4
#define PX4_CUSTOM_AUTO_SUBMODE_LOITER 3

extern mavlink_system_t mavlink_system;
extern mavlink_system_t mavlink_autopilot;

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

void set_pos_freq(int freq);

ListNode * queue_message(mavlink_message_t * msg, uint32_t procID);

// IMPORTANT: only meant to be called by main process in CM7
uint8_t send_command(uint32_t cmdid, mavlink_message_t * msg);
uint8_t send_command_int(uint16_t command, uint8_t frame,
													float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z);
uint8_t send_command_long(uint16_t command,
													float param1, float param2, float param3, float param4, float param5, float param6, float param7);

// send ping message
uint8_t send_ping_message(void);

// arm/disarm
// armval: 1 to arm, 0 to disarm
// force: 1 to force state, 0 to not
uint8_t send_arm_disarm_message(uint8_t armval, uint8_t force);

uint8_t set_pos_setpoint(uint32_t procID, uint8_t frame, uint16_t mask,
		float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float rate);
uint8_t set_offboard(uint32_t procID);

uint8_t set_pos_hold(uint32_t procID);
uint8_t set_hold_mode(uint32_t procID);
uint8_t set_vel_hold(uint32_t procID);

uint8_t takeoff(float meters);

uint8_t set_flower_setpoint(void);

#endif /* INC_MAVLINK_PSSC_H_ */
