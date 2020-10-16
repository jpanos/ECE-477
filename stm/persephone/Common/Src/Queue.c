/*
 * Queue
 *
 *  Created on: Oct 9, 2020
 *      Author: moiz
 */
#include <Queue.h>

uint8_t init_queue(Queue * q) {
	q->tail = NULL;
	q->head = NULL;
	return 0;
}

uint8_t enqueue(Queue * q, ListNode * e) {
	e->next = NULL;
	if (q->tail == NULL) {
		q->head = e;
		q->tail = e;
		return 0;
	}
	q->tail->next = e;
	q->tail = e;
	return 0;
}

ListNode * dequeue(Queue * q) {
	if (q->head == NULL) return NULL;
	ListNode * r = q->head;
	q->head = q->head->next;
	if (q->head == NULL) q->tail = NULL;
	r->next = NULL;
	return r;
}
