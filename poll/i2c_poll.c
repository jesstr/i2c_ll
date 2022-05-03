#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#include "stm32l0xx.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_cortex.h"

#define I2C_TIMEOUT_MS              5

/* Set to '0' if some part of HW are initialized externally */
#define I2C_INIT_HW                 0 /* '0' - Skip HW initialization */
#define I2C_INIT_GPIO               0 /* '0' - Skip GPIO initialization */

#if I2C_INIT_GPIO
#define I2C_SCL_PIN                 LL_GPIO_PIN_8
#define I2C_SDA_PIN                 LL_GPIO_PIN_9
#define I2C_GPIO                    GPIOB
#define I2C_GPIO_RCC                LL_IOP_GRP1_PERIPH_GPIOB
#endif

typedef enum {
    I2C_TRANSMITTER,
    I2C_RECEIVER,
    I2C_RECEIVER_RESTART
} i2c_direction_t;


void I2Cx_Init(I2C_TypeDef* I2Cx)
{
 
#if I2C_INIT_HW
    /* Periph clock enable */
    if (I2Cx == I2C1) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    }
    else if (I2Cx == I2C2) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
    }
    else {
        return;
    }

#if I2C_INIT_GPIO
    /* NOTE! External pullups are required! */
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    LL_GPIO_StructInit(&GPIO_InitStruct);

    LL_IOP_GRP1_EnableClock(I2C_GPIO_RCC);

    GPIO_InitStruct.Pin = I2C_SCL_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
    LL_GPIO_Init(I2C_GPIO, &GPIO_InitStruct);
#endif /* I2C_INIT_GPIO */

    LL_I2C_DeInit(I2Cx);

    /* I2C configuration */
    LL_I2C_InitTypeDef I2C_InitStruct = {0};
    LL_I2C_StructInit(&I2C_InitStruct);
    
    I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
    /* This parameter should be calculated by the STM32CubeMX tool */
    I2C_InitStruct.Timing = 0x00200C28;
    I2C_InitStruct.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
    I2C_InitStruct.DigitalFilter = 0;
    I2C_InitStruct.OwnAddress1 = 0;
    I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
    I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;

    LL_I2C_EnableAutoEndMode(I2Cx);
    LL_I2C_DisableOwnAddress2(I2Cx);
    LL_I2C_DisableGeneralCall(I2Cx);
    LL_I2C_EnableClockStretching(I2Cx);

    LL_I2C_Init(I2Cx, &I2C_InitStruct);
    LL_I2C_SetOwnAddress2(I2Cx, 0, LL_I2C_OWNADDRESS2_NOMASK);
#endif /* I2C_INIT_HW */
}


static bool I2Cx_StartTransmission(I2C_TypeDef* I2Cx, i2c_direction_t Direction,
    uint8_t SlaveAddr, uint8_t TransferSize)
{
    uint32_t Timeout = I2C_TIMEOUT_MS;
    uint32_t Request;

    switch (Direction) {
    case I2C_TRANSMITTER:
        Request = LL_I2C_GENERATE_START_WRITE;
        break;
    case I2C_RECEIVER:
        Request = LL_I2C_GENERATE_START_READ;
        break;
    case I2C_RECEIVER_RESTART:
        Request = LL_I2C_GENERATE_RESTART_7BIT_READ;
        break;
    default:
        return false;
    }

    LL_I2C_HandleTransfer(I2Cx, SlaveAddr, LL_I2C_ADDRSLAVE_7BIT,
        TransferSize, LL_I2C_MODE_SOFTEND, Request);

    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx) && !LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
        if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
            LL_I2C_ClearFlag_NACK(I2Cx);
            return false;
        }
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (Timeout-- == 0) {
                return false;
            }
        }
    }

    return true;
}


static bool I2Cx_SendByte(I2C_TypeDef* I2Cx, uint8_t byte)
{
    uint32_t Timeout = I2C_TIMEOUT_MS;

    LL_I2C_TransmitData8(I2Cx, byte);
    while (!LL_I2C_IsActiveFlag_TXIS(I2Cx) && !LL_I2C_IsActiveFlag_TC(I2Cx)) {
        /* Break if ACK failed */
        if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
            LL_I2C_ClearFlag_NACK(I2Cx);
            return false;
        }
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (Timeout-- == 0) {
                return false;
            }
        }
    }

    return true;
}


static bool I2Cx_ReceiveByte(I2C_TypeDef* I2Cx, uint8_t *Byte)
{
    uint32_t Timeout = I2C_TIMEOUT_MS;

    if (!Byte) return false;

    while (!LL_I2C_IsActiveFlag_RXNE(I2Cx) && !LL_I2C_IsActiveFlag_TC(I2Cx)) {
        if (LL_SYSTICK_IsActiveCounterFlag()) {
            if (Timeout-- == 0) {
                return false;
            }
        }
    }

    *Byte = LL_I2C_ReceiveData8(I2Cx);

    return true;
}


static void I2Cx_StopTransmission(I2C_TypeDef* I2Cx)
{
    /* Send STOP bit */
    LL_I2C_GenerateStopCondition(I2Cx);
}


bool I2Cx_ReadData(I2C_TypeDef* I2Cx, uint8_t SlaveAddr, uint16_t ReadAddr,
    uint8_t AddrLen, uint8_t *pBuffer, uint8_t NumBytesToRead)
{
    if (!pBuffer || !NumBytesToRead) {
        return false;
    }

    SlaveAddr = SlaveAddr << 1;

    if (AddrLen) {
        if (!I2Cx_StartTransmission(I2Cx, I2C_TRANSMITTER, SlaveAddr, AddrLen)) {
            return false;
        }
        if (AddrLen == 2 && !I2Cx_SendByte(I2Cx, ReadAddr >> 8)) {
            I2Cx_StopTransmission(I2Cx);
            return false;
        }
        if (!I2Cx_SendByte(I2Cx, ReadAddr & 0xFF)) {
            I2Cx_StopTransmission(I2Cx);
            return false;
        }
    }

    if (!I2Cx_StartTransmission(I2Cx, AddrLen ? I2C_RECEIVER_RESTART : I2C_RECEIVER, SlaveAddr, NumBytesToRead)) {
        return false;
    }

    while (NumBytesToRead--) {
        if (!I2Cx_ReceiveByte(I2Cx, pBuffer++)) {
            I2Cx_StopTransmission(I2Cx);
            return false;
        }
        if (NumBytesToRead == 0) {
            /* Send STOP after the last byte */
            I2Cx_StopTransmission(I2Cx);
        }
    }

    return true;
}


bool I2Cx_WriteData(I2C_TypeDef* I2Cx, uint8_t SlaveAddr, uint16_t WriteAddr,
    uint8_t AddrLen, uint8_t *pBuffer, uint8_t NumBytesToWrite)
{
    bool Result = true;

    if (!pBuffer || !NumBytesToWrite) {
        return false;
    }

    SlaveAddr = SlaveAddr << 1;

    if (!I2Cx_StartTransmission(I2Cx, I2C_TRANSMITTER, SlaveAddr, AddrLen + NumBytesToWrite)) {
        return false;
    }

    if (AddrLen == 2 && !I2Cx_SendByte(I2Cx, WriteAddr >> 8)) {
        I2Cx_StopTransmission(I2Cx);
        return false;
    }
    if (AddrLen >= 1 && !I2Cx_SendByte(I2Cx, WriteAddr & 0xFF)) {
        I2Cx_StopTransmission(I2Cx);
        return false;
    }

    while (NumBytesToWrite--) {
        if (!I2Cx_SendByte(I2Cx, *pBuffer++)) {
            Result = false;
            break;
        }
    }
    I2Cx_StopTransmission(I2Cx);

    return Result;
}