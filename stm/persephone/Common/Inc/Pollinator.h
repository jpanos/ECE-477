/*
 * Pollinator.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Josh Panos
 */

#ifndef INC_POLLINATOR_H_
#define INC_POLLINATOR_H_

#define POLLINATOR_STRAIGHT_ANGLE   150
#define POLLINATOR_UP_ANGLE         140
#define POLLINATOR_DOWN_ANGLE       160
#define POLLINATOR_DOWN_RANGE_ANGLE 180

#define POLLINATOR_POS_X 10
#define POLLINATOR_POS_Y 25
#define POLLINATOR_POS_Z 405

int check_sense(void);
void set_angle(int);
int get_angle(void);
void set_angle_deg(int deg);
int get_angle_deg(void);

void init_servoPWM(void);
void init_touchISR(void);

#endif /* INC_POLLINATOR_H_ */
