#include "main.h"

void initI2C(void);
void I2C1GPIOINIT(void);
void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction);