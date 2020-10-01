/*
 * nano_uart.c
 *
 *  Created on: Sep 28, 2020
 *      Author: ethan
 */
#include "main.h"
#include "nano_uart.h"

int initFlowers(t_flower * flowers, char numFlowers){ //does numFlowers need to be an int?
  for(int i; i < numFlowers; i++){
    flowers[i].id = FLOWER_DEFUALT_ID;
    flowers[i].x = FLOWER_DEFAULT_X;
    flowers[i].y = FLOWER_DEFAULT_Y;
    flowers[i].z = FLOWER_DEFAULT_Z;
    flowers[i].conf = FLOWER_DEFAULT_CONF;
  }
  return 1;
}

int getFlowers(t_flower * flowers, int numFlowers, UART_HandleTypeDef * huart){
  //handshake
  char bytePacket[] = {NANO_COMM_ON_OFF};
  HAL_UART_Transmit(huart, bytePacket, sizeof(bytePacket), 300);
  //wait for the ready flag
  while(!(USART1->ISR & USART_ISR_RXNE_RXFNE)){}
  HAL_UART_Receive(huart, bytePacket, sizeof(bytePacket), 300);

  if(bytePacket[0] != NANO_COMM_ON){return 0;}
  else{ //correct response
    bytePacket[0] = NANO_COMM_TYPE_FLOWER; //change the packet to a flower request
  }

  HAL_UART_Transmit(huart, bytePacket, sizeof(bytePacket), 300);
  while(!(USART1->ISR & USART_ISR_RXNE_RXFNE)){}
  HAL_UART_Receive(huart, bytePacket, sizeof(bytePacket), 300);

  if(bytePacket[0] != NANO_COMM_ON){return 0;}
  else{ bytePacket[0] = numFlowers; }

  HAL_UART_Transmit(huart, bytePacket, sizeof(bytePacket), 300);
  while(!(USART1->ISR & USART_ISR_RXNE_RXFNE)){}
  HAL_UART_Receive(huart, flowers, sizeof(t_flower) * numFlowers, 300);

  char str_buffer[] = "ID | X | Y | Z | CONF\n";
  HAL_UART_Transmit(huart, str_buffer, sizeof(str_buffer), 300);

  HAL_UART_Transmit(huart, flowers, sizeof(t_flower) * numFlowers, 300);

  return 1;
}

