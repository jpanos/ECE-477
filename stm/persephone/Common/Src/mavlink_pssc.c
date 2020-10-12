#include <mavlink_pssc.h>


mavlink_system_t mavlink_system = {
		1, // System ID (0-255)
		MAV_COMP_ID_ONBOARD_COMPUTER // Computer ID (a MAV_COMPONENT value)
};

volatile MavlinkSharedData * const mv_shared = (MavlinkSharedData *) MVPSSC_SHARED_SRAM_LOC;

uint8_t queue_message(mavlink_message_t * msg, uint32_t procID) {
	ListNode * n;
	spin_lock(HSEM_ID_EMPTY, procID);
	n = dequeue(&mv_shared->empty_nodes);
	lock_release(HSEM_ID_EMPTY, procID);

	if (n == NULL) return MVPSSC_FAIL;

	MsgData * msgdata = (MsgData *) n->data;

	mavlink_msg_to_send_buffer(&msgdata->buff, msg);
	msgdata->len = msg->len + 12;

	spin_lock(HSEM_ID_MSGS, procID);
	enqueue(&mv_shared->send_msgs, n);
	lock_release(HSEM_ID_MSGS, procID);

	return MVPSSC_SUCCESS;
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
	return queue_message(&msg, 0);
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
	return queue_message(&msg, 0);
}

uint8_t send_arm__disarm_message(uint8_t armval, uint8_t force) {
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
			1, //arm
			21196, //force
			0, 0, 0, 0, 0);
	return queue_message(&msg, 0);
}


