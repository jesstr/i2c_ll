#ifndef I2C_POLL_H
#define I2C_POLL_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32l0xx.h"
#include "stm32l0xx_ll_i2c.h"

void I2Cx_Init(I2C_TypeDef* I2Cx);

bool I2Cx_ReadData(I2C_TypeDef* I2Cx, uint8_t SlaveAddr, uint16_t ReadAddr,
    uint8_t AddrLen, uint8_t *pBuffer, uint8_t NumBytesToRead);
bool I2Cx_WriteData(I2C_TypeDef* I2Cx, uint8_t SlaveAddr, uint16_t WriteAddr,
    uint8_t AddrLen, uint8_t *pBuffer, uint8_t NumBytesToWrite);

#endif /* I2C_POLL_H */