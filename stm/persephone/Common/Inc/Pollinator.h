/*
 * Pollinator.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Josh Panos
 */

#ifndef INC_POLLINATOR_H_
#define INC_POLLINATOR_H_

int check_sense(void);
void set_angle(int);
int get_angle(void);
void set_angle_deg(int deg);
int get_angle_deg(void);


void init_servoPWM(void);
void init_touchISR(void);

#endif /* INC_POLLINATOR_H_ */
