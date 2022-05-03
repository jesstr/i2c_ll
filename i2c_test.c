#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32l0xx.h"
#include "i2c_ll.h"

#define I2C_EMPTY_ADDR  0x41

#define I2C_VALID_ADDR  0x40
#define I2C_VALID_REG   0xFE
#define I2C_VALID_DATA  0x54, 0x49
#define I2C_VALID_LEN   2


bool I2Cx_Test(I2C_TypeDef* I2Cx)
{
    uint8_t buf[I2C_VALID_LEN] = {0};
    uint8_t ref[] = {I2C_VALID_DATA};

    /* Read empty address, expected NACK */
    if (I2Cx_ReadData(I2C1, I2C_EMPTY_ADDR, 0x00, 0, buf, 1)) return false;
    if (I2Cx_ReadData(I2C1, I2C_EMPTY_ADDR, 0x01, 1, buf, 2)) return false;
    /* Write empty address, expected NACK */
    if (I2Cx_WriteData(I2C1, I2C_EMPTY_ADDR, 0x00, 0, buf, 1)) return false;
    if (I2Cx_WriteData(I2C1, I2C_EMPTY_ADDR, 0x01, 1, buf, 2)) return false;

    /* Read data from valid address, expected ACK */
    for (uint8_t i = 1; i <= I2C_VALID_LEN; i++) {
        if (!I2Cx_ReadData(I2C1, I2C_VALID_ADDR, I2C_VALID_REG, 1, buf, i)) return false;
        if (memcmp(buf, ref, i) != 0) return false;
    }

    return true;
}