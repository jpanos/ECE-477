/*
 * nano_uart.h
 *
 *  Created on: Sep 28, 2020
 *      Author: ethan
 */

#ifndef INC_NANO_UART_H_
#define INC_NANO_UART_H_

#include "stm32h7xx_hal_uart.h"

#define SIZE_FLOWER_ARRAY 100
#define FLOWER_DEFUALT_ID 0
#define FLOWER_DEFAULT_X 0
#define FLOWER_DEFAULT_Y 0
#define FLOWER_DEFAULT_Z 0
#define FLOWER_DEFAULT_CONF 0

#define NANO_COMM_ON_OFF 0x01
#define NANO_COMM_ON 0x02
#define NANO_COMM_OFF 0x03
#define NANO_COMM_TYPE_FLOWER 0x06

typedef struct flw {
  int id;
  int x;
  int y;
  int z;
  int conf;
} t_flower;


t_flower g_flowers[SIZE_FLOWER_ARRAY]; //global flower array

/**
 * @brief clear the array of flowers to default values
 * @param flowers: array of t_flower to modify.
 * @param numFlowers: number of flowers to init
 * @return: 1 if success, 0 if fail
 */
int initFlowers(t_flower * flowers, char numFlowers);

/**
 * @brief get flowers from the given uart peripheral
 * @param flowers: the array of flowers that will be filled
 * @param numFlowers: the number of flowers to request
 * @param huart: the pointer to uart peripheral
 * @return: 1 if success, 0 if fail
 */
int getFlowers(t_flower * flowers, int numFlowers, UART_HandleTypeDef * huart);

#endif /* INC_NANO_UART_H_ */
