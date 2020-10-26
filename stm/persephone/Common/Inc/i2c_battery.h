/* I2C initialization header for the battery monitor
By Jackie
*/
#include "main.h"

#define MASTERWRITE 0
#define MASTERREAD  1

#ifndef INC_I2C_BATTERY_H
#define INC_I2C_BATTERY_H

void initI2C2(void);
void initI2C1(void);
void I2C1GPIOINIT(void);
void I2C2GPIOINIT(void);
void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction);
void I2CreadfromBatt(int addr);
//int I2C_getCellVoltaddr(char cellnum);

#endif /* INC_I2C_BATTERY_H */
