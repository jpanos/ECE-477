/* I2C initialization header for the battery monitor
By Jackie
*/
#include "main.h"

#define MASTERWRITE 0
#define MASTERREAD  1

#ifndef INC_I2C_BATTERY_H
#define INC_I2C_BATTERY_H

void initI2C2(void);
void I2C2GPIOINIT(void);
void I2C_StartTX(I2C_TypeDef* I2C, uint32_t DevAddress, uint8_t Size, uint8_t Direction);
void I2C2battTalk(int writeMode, int regAddr, char byte);
void computeVoltages(void);
void storeVData(void);
void getBatPercent(void);
void initUART(void);

extern float batValLUT[100];

#endif /* INC_I2C_BATTERY_H */
