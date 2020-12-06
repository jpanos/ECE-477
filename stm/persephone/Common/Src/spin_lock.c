/*
 * spin_lock.c
 *
 *  Created on: Oct 10, 2020
 *      Author: moiz
 */
#include <spin_lock.h>

HAL_StatusTypeDef spin_lock(uint32_t SemID, uint32_t ProcessID) {
	HAL_StatusTypeDef retval = HAL_ERROR;
	while (retval == HAL_ERROR) {
		retval = HAL_HSEM_Take(SemID, ProcessID);
	}
	return retval;
}

