/*
 * Queue
 *
 *  Created on: Oct 9, 2020
 *      Author: moiz
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#include <stm32h7xx_hal.h>

typedef struct _linked_list_node {
	struct _linked_list_node * next;
	void * data;
} ListNode;

typedef struct _queue {
	ListNode * head;
	ListNode * tail;
} Queue;

uint8_t init_queue(Queue * q);

uint8_t enqueue(Queue * q, ListNode * e);

ListNode * dequeue(Queue * q);

#endif /* INC_QUEUE_H_ */
