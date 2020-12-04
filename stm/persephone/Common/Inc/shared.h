/*
 * shared.h
 *
 *  Created on: Oct 22, 2020
 *      Author: moiz
 */

#ifndef INC_SHARED_H_
#define INC_SHARED_H_

#include <queue.h>
#include <mavlink_pssc.h>

#define SHARED_SRAM_LOC 0x30040000

// shared data semaphore IDs
#define HSEM_ID_EMPTY 15
#define HSEM_ID_MSGS 16
#define HSEM_ID_CMD 17
#define HSEM_ID_CMD_BLOCK HSEM_ID_CMD
#define HSEM_ID_START_OFFBOARD_FLAG 18
#define HSEM_ID_POS_SETPOINT 19
#define HSEM_ID_FLOWER_POS_DATA 20

// shared data process IDs
#define DMA1_S0_PROC_ID 40
#define TIM6_PROC_ID 41
#define UART1_RX_PROC_ID 42
#define UART3_RX_PROC_ID 43

typedef struct _point{
	float x;
	float y;
	float z;
} Point;

// a struct to hold all shared data between both cores
// we are limited to size of SRAM3 which is ~32KB
typedef struct _mavlink_shared_data {

//////////////SYSTEM DATA START/////////////////////
	uint32_t time_boot_ms;
//////////////SYSTEM DATA END///////////////////////

//////////////MAVLINK SHARED DATA START/////////////////////
	uint8_t mav_state;
	uint8_t mav_mode_flag;
	uint8_t landed_state;

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

	float pos_x;
	float pos_y;
	float pos_z;

	float idle_height;

	int pos_period;
	uint8_t pos_mode;
	uint8_t pos_coord_frame;
	uint16_t pos_type_mask;
	float pos_set_x;
	float pos_set_y;
	float pos_set_z;
	float pos_set_vx;
	float pos_set_vy;
	float pos_set_vz;
	float pos_set_afx;
	float pos_set_afy;
	float pos_set_afz;
	float pos_set_yaw;
	float pos_set_yaw_rate;
//////////////MAVLINK SHARED DATA END//////////////////////

//////////////I2c shit	/////////////////////
	char masterrxdata;
	char mastertxdata;
	int i2cTargReg;
	char i2cmode;
	int regReading;
	int VC1;
	int VC2;
	int VC3;
	int VC4;
	int VC5;
	float cell1;
	float cell2;
	float cell3;
	float cell4;
	float total;
	int bat;
	char computeVoltageFlag;
	char count;
	float voltage;
	int batPercentRemain;

// usart stuff
	Point flowercoord;
	char usartct;
	uint8_t usartbuff[12];
} MavlinkSharedData;

extern volatile MavlinkSharedData * const shared;

// should prob only be called by one core
uint8_t shared_init();

#endif /* INC_SHARED_H_ */
