/*
 * shared.c
 *
 *  Created on: Oct 22, 2020
 *      Author: moiz
 */

#include <shared.h>

volatile MavlinkSharedData * const shared = (MavlinkSharedData *) SHARED_SRAM_LOC;

uint8_t shared_init() {
	shared->time_boot_ms = 0;
}
