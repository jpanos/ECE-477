#include <mavlink_pssc.h>


mavlink_system_t mavlink_system = {
		1, // System ID (0-255)
		MAV_COMP_ID_ONBOARD_COMPUTER // Computer ID (a MAV_COMPONENT value)
};

volatile MavlinkSharedData * const mv_shared = (MavlinkSharedData *) MVPSSC_SHARED_SRAM_LOC;

ListNode * queue_message(mavlink_message_t * msg, uint32_t procID) {
	ListNode * n;
	spin_lock(HSEM_ID_EMPTY, procID);
	n = dequeue(&mv_shared->empty_nodes);
	lock_release(HSEM_ID_EMPTY, procID);

	if (n == NULL) return n;

	MsgData * msgdata = (MsgData *) n->data;

	mavlink_msg_to_send_buffer(&msgdata->buff, msg);
	msgdata->len = msg->len + 12;

	spin_lock(HSEM_ID_MSGS, procID);
	enqueue(&mv_shared->send_msgs, n);
	lock_release(HSEM_ID_MSGS, procID);

	return n;
}

uint8_t send_command(uint32_t cmdid, mavlink_message_t * msg) {
	ListNode * n = NULL;
	mv_shared->cmd.done = MVPSSC_CMD_START;

	while (n == NULL) {
	 n = queue_message(msg, 0);
	}
	mv_shared->cmd.node = n;
	mv_shared->cmd.cmd_id = cmdid;

	while (mv_shared->cmd.done != MVPSSC_CMD_DONE) {}

	// mv_shared->cmd.node = NULL;

	return mv_shared->cmd.result;
}


uint8_t set_mavlink_msg_interval(uint16_t message_id, int32_t interval_us) {
	// given a message id and us interval, device that is sent this command will send mesage at that interval
	mavlink_message_t msg;
	mavlink_msg_message_interval_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			message_id,
			interval_us
			);
	if (queue_message(&msg, 0) != NULL) return MVPSSC_SUCCESS;
	else return MVPSSC_FAIL;
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

	mavlink_message_t msg;
	mavlink_msg_command_int_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			mavlink_system.sysid,
			MAV_COMP_ID_AUTOPILOT1,
			MAV_FRAME_MISSION,
			MAV_CMD_COMPONENT_ARM_DISARM,
			1,
			1,
			armval, //arm
			forceval, //force
			0, 0, 0, 0, 0);
	return send_command(MAV_CMD_COMPONENT_ARM_DISARM, &msg);
}

uint8_t send_command_int(uint16_t command, uint8_t frame,
													float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z) {
	mavlink_message_t msg;
	mavlink_msg_command_int_pack(
			mavlink_system.sysid,
			mavlink_system.compid,
			&msg,
			mavlink_system.sysid,
			MAV_COMP_ID_AUTOPILOT1,
			frame,
			command,
			1, // current
			0, // autocontinue
			param1, param2, param3, param4, x, y, z);
	return send_command(command, &msg);
}

