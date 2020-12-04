#include <mavlink_pssc.h>
#include <stm32h7xx_hal.h>
#include <common/mavlink.h>
#include <spin_lock.h>
#include <queue.h>
#include <shared.h>
#include <Pollinator.h>

mavlink_system_t mavlink_system = {
		1, // System ID (0-255)
		MAV_COMP_ID_ONBOARD_COMPUTER // Computer ID (a MAV_COMPONENT value)
};

mavlink_system_t mavlink_autopilot = {
		1,
		1
};

// value is limited by max frequency of TIM6 timer interrupt generation
// current max is 100Hz, min value is 2Hz as per spec by px4
// will round down if not evenly divisible into 100
void set_pos_freq(int freq) {
	int period = 1000 / freq;
	shared->pos_period = period - period % 10;
}

ListNode * queue_message(mavlink_message_t * msg, uint32_t procID) {
	ListNode * n;
	spin_lock(HSEM_ID_EMPTY, procID);
	n = dequeue(&shared->empty_nodes);
	lock_release(HSEM_ID_EMPTY, procID);

	if (n == NULL) return n;

	MsgData * msgdata = (MsgData *) n->data;

	mavlink_msg_to_send_buffer(&msgdata->buff, msg);
	msgdata->len = msg->len + 12;

	spin_lock(HSEM_ID_MSGS, procID);
	enqueue(&shared->send_msgs, n);
	lock_release(HSEM_ID_MSGS, procID);

	return n;
}

uint8_t send_command(uint32_t cmdid, mavlink_message_t * msg) {
	ListNode * n = NULL;
	shared->cmd.done = MVPSSC_CMD_START;

	while (n == NULL) {
	 n = queue_message(msg, 0);
	}
	shared->cmd.node = n;
	shared->cmd.cmd_id = cmdid;

	while (shared->cmd.done != MVPSSC_CMD_DONE) {}

	// shared->cmd.node = NULL;

	return shared->cmd.result;
}


uint8_t send_ping_message(void) {
	mavlink_message_t msg;
	mavlink_msg_ping_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			10000,
			40,
			0, 0); // all systems and components listening
	if (queue_message(&msg, 0) != NULL) return MVPSSC_SUCCESS;
	else return MVPSSC_FAIL;
}

uint8_t send_arm_disarm_message(uint8_t arm, uint8_t force) {
	int forceval = 0;
	int armval = 0;

	if (force) forceval = 21196;
	if (arm) armval = 1;

	return send_command_long(MAV_CMD_COMPONENT_ARM_DISARM, armval, forceval, 0, 0, 0, 0, 0);
}

uint8_t send_command_int(uint16_t command, uint8_t frame,
													float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z) {
	mavlink_message_t msg;
	mavlink_msg_command_int_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			mavlink_autopilot.sysid,
			mavlink_autopilot.compid,
			frame,
			command,
			1, // current
			0, // autocontinue
			param1, param2, param3, param4, x, y, z);
	return send_command(command, &msg);
}

uint8_t send_command_long(uint16_t command,
													float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
	mavlink_message_t msg;
	mavlink_msg_command_long_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			mavlink_autopilot.sysid,
			mavlink_autopilot.compid,
			command,
			0,
			param1, param2, param3, param4, param5, param6, param7);
	return send_command(command, &msg);
}

uint8_t set_pos_setpoint(uint32_t procID, uint8_t frame, uint16_t mask,
		float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float rate) {
	spin_lock(HSEM_ID_POS_SETPOINT, procID);
	shared->pos_coord_frame = frame;
	shared->pos_type_mask = mask;
	shared->pos_set_x = x;
	shared->pos_set_y = y;
	shared->pos_set_z = z;
	shared->pos_set_vx = vx;
	shared->pos_set_vy = vy;
	shared->pos_set_vz = vz;
	shared->pos_set_afx = afx;
	shared->pos_set_afy = afy;
	shared->pos_set_afz = afz;
	shared->pos_set_yaw = yaw;
	shared->pos_set_yaw_rate = rate;
	lock_release(HSEM_ID_POS_SETPOINT, procID);
}

uint8_t set_offboard(uint32_t procID) {
	uint8_t result = MAV_RESULT_FAILED;
	send_command_long(MAV_CMD_NAV_GUIDED_ENABLE, 0, 0, 0, 0, 0, 0, 0);
	while (result != MAV_RESULT_ACCEPTED) {
		result = send_command_long(MAV_CMD_DO_SET_MODE, PX4_MODE, PX4_CUSTOM_MODE_OFFBOARD, 0, 0, 0, 0, 0);
	}
	return MVPSSC_SUCCESS;
}

uint8_t set_pos_hold(uint32_t procID) {
	set_pos_setpoint(procID, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_POSITION_SETPOINT, shared->pos_x, shared->pos_y, shared->pos_z,
										0, 0, 0, 0, 0, 0, 0, 0);
	return MVPSSC_SUCCESS;
}

uint8_t set_hold_mode(uint32_t procID) {
	send_command_long(MAV_CMD_DO_SET_MODE,
										PX4_MODE,
										PX4_CUSTOM_MODE_AUTO, PX4_CUSTOM_AUTO_SUBMODE_LOITER, 0, 0, 0, 0);
	set_pos_setpoint(procID, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_POSITION_SETPOINT, shared->pos_x, shared->pos_y, shared->pos_z,
										0, 0, 0, 0, 0, 0, 0, 0);
	return MVPSSC_SUCCESS;
}

uint8_t set_vel_hold(uint32_t procID) {
	set_pos_setpoint(procID, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0);
	return MVPSSC_SUCCESS;
}

uint8_t takeoff(float meters) {
  float z_setpoint = shared->pos_z - meters;
  set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, 0, 0, -.7, 0, 0, 0, 0, 0);
  while (shared->pos_z > z_setpoint) {}
  set_pos_setpoint(0, MAV_FRAME_LOCAL_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  return MVPSSC_SUCCESS;
}

float intsgn(int i) {
  return (float) ((i > 0) - (i < 0));
}

uint8_t set_flower_setpoint(uint32_t procID) {
  if (!(shared->pos_mode & MVPSSC_POS_MODE_FLOWER)) return MVPSSC_FAIL;
  float vx = 0,vy = 0,vz = 0;

  // +z axis of flowers is +x axis in body coordinate frame of drone
  int diffx = shared->flowercoord.z - POLLINATOR_POS_Z;
  if (diffx < 0) diffx = 0;
  // +x axis of flowers is +y axis in body coordinate frame of drone
  int diffy = shared->flowercoord.x - POLLINATOR_POS_X;
  // +y axis of flowers is +z axis in body coordinate frame of drone
  int diffz = shared->flowercoord.y - POLLINATOR_POS_Y;


  if (abs(diffx) > MVPSSC_POS_X_ERR) vx = .1 * intsgn(diffx);
  if (abs(diffy) > MVPSSC_POS_Y_ERR) vy = .1 * intsgn(diffy);
  // if (abs(diffz) > MVPSSC_POS_Z_ERR) vz = .1 * intsgn(diffz);

//  if (shared->pos_x > shared->idle_height) vz = -.1;

  if (abs(diffx) < MVPSSC_POS_X_ERR && abs(diffy) < MVPSSC_POS_Y_ERR) {
    shared->pos_mode |= MVPSSC_POS_MODE_LAND;
    shared->pos_mode &= ~MVPSSC_POS_MODE_FLOWER;
  }
  // shared->pos_mode |= MVPSSC_POS_MODE_LAND;

  set_pos_setpoint(procID, MAV_FRAME_BODY_NED, MVPSSC_POS_MASK_VELOCITY_SETPOINT,
      0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0);

  return MVPSSC_SUCCESS;
}

