/* I2C initialization header for the battery monitor
By Jackie
*/
#include "main.h"

#ifndef INC_I2C_BATTERY_H
#define INC_I2C_BATTERY_H

void initI2C(void);
void I2C1GPIOINIT(void);
void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction);

#endif /* INC_I2C_BATTERY_H */
