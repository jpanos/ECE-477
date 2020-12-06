/*
 * spin_lock.h
 *
 *  Created on: Oct 10, 2020
 *      Author: moiz
 */

#ifndef INC_SPIN_LOCK_H_
#define INC_SPIN_LOCK_H_

#include <stm32h7xx_hal.h>

HAL_StatusTypeDef spin_lock(uint32_t SemID, uint32_t ProcessID);
#define lock_release(SemID, ProcessID) HAL_HSEM_Release(SemID, ProcessID)

#endif /* INC_SPIN_LOCK_H_ */
