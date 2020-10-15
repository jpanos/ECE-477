#include "main.h"

#ifndef __i2c_H
#define __i2c_H
#ifdef __cplusplus
 extern "C" {
#endif

void initI2C(void);
void I2C1GPIOINIT(void);
void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction);

#ifdef __cplusplus
}
#endif
#endif 