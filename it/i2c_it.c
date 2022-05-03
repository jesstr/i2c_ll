#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "i2c.h"

#include "stm32l0xx.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_cortex.h"

#include "FreeRTOS.h"
#include "semphr.h"

#undef DBG
#define DBG if(0)

#define I2C_SPEED_FREQ                          400000
#define I2C_BYTE_TIMEOUT_US                     ((10^6) / (I2C_SPEED_FREQ / 9) + 1)
#define I2C_TRANSFER_MIN_TIMEOUT_MS             2

/* For a simlicity START + ADDR(R/W) + RESTART + STOP
 * are considered as a single 2-byte transaction */
#define I2C_TRANSFER_TIMEOUT_MS(num_bytes)      (((2 + num_bytes) * 9 * I2C_BYTE_TIMEOUT_US) / \
                                                    1000 + I2C_TRANSFER_MIN_TIMEOUT_MS)

/* Set to '0' if some part of HW are initialized externally */
#define I2C_INIT_HW                 0 /* '0' - Skip HW initialization */
#define I2C_INIT_GPIO               0 /* '0' - Skip GPIO initialization */

#if I2C_INIT_GPIO
#define I2C_SCL_PIN                 LL_GPIO_PIN_8
#define I2C_SDA_PIN                 LL_GPIO_PIN_9
#define I2C_GPIO                    GPIOB
#define I2C_GPIO_RCC                LL_IOP_GRP1_PERIPH_GPIOB
#endif

#define I2C_NUM_IFACES              2

#define IFACE_NUM(X)                (X == I2C1 ? 0 : \
                                     X == I2C2 ? 1 : 0xFF)

typedef enum {
    RESTART,
    CONTINUE,
    FINISHED,
    FAILED
} i2c_state_t;

typedef enum {
    I2C_TRANSMITTER,
    I2C_RECEIVER
} i2c_direction_t;


static struct IfaceStruct
{
    uint8_t SlaveAddr;
    i2c_direction_t Direction;
    struct {
        uint16_t Len;
        uint16_t Idx;
        uint8_t *Buf;
    } Rx;
    struct Tx {
        uint16_t Len;
        uint16_t Idx;
        uint8_t *Buf;
    } Tx;
    struct {
        uint16_t Val;
        uint8_t Len;
        uint8_t Idx;
    } Addr;
} Ifaces[I2C_NUM_IFACES];

static SemaphoreHandle_t xSemphr;
static SemaphoreHandle_t xMutex;


static struct IfaceStruct *GetIface(I2C_TypeDef* I2Cx)
{
    uint8_t n_iface;

    if ((n_iface = IFACE_NUM(I2Cx)) >= I2C_NUM_IFACES) {
        return NULL;
    }

    return &Ifaces[n_iface];
}


static void ResetIface(struct IfaceStruct *Iface)
{
    Iface->Addr.Idx = 0;
    Iface->Addr.Len = 0;

    Iface->Rx.Idx = 0;
    Iface->Rx.Len = 0;
    Iface->Rx.Buf = NULL;

    Iface->Tx.Idx = 0;
    Iface->Tx.Len = 0;
    Iface->Tx.Buf = NULL;
}


void I2Cx_Init(I2C_TypeDef* I2Cx)
{
    xSemphr = xSemaphoreCreateBinary();
    xMutex = xSemaphoreCreateMutex();

#if I2C_INIT_HW
    IRQn_Type IRQn;

    /* Periph clock enable */
    if (I2Cx == I2C1) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
        IRQn = I2C1_IRQn;
    }
    else if (I2Cx == I2C2) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C2);
        IRQn = I2C2_IRQn;
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

    /* Interrupts configuration */
    NVIC_SetPriority(IRQn, 3);
    NVIC_EnableIRQ(IRQn);
#endif /* I2C_INIT_HW */
}


static bool I2Cx_StartTransmission(I2C_TypeDef* I2Cx, i2c_direction_t Direction,
    uint8_t SlaveAddr, uint8_t TransferSize)
{
    struct IfaceStruct *Iface = GetIface(I2Cx);

    if (!Iface) {
        return false;
    }

    Iface->SlaveAddr = SlaveAddr << 1;
    Iface->Direction = Direction;

    LL_I2C_EnableIT_TX(I2Cx);
    LL_I2C_EnableIT_TC(I2Cx);

    LL_I2C_HandleTransfer(I2Cx, Iface->SlaveAddr, LL_I2C_ADDRSLAVE_7BIT,
        TransferSize, LL_I2C_MODE_SOFTEND, Direction == I2C_TRANSMITTER ?
            LL_I2C_GENERATE_START_WRITE : LL_I2C_GENERATE_START_READ);

    return true;
}


static void I2Cx_StopTransmissionISR(I2C_TypeDef* I2Cx, bool GenerateStop)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Send STOP bit if required */
    if (GenerateStop) {
        LL_I2C_GenerateStopCondition(I2Cx);
    }

    xSemaphoreGiveFromISR(xSemphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


bool I2Cx_ReadData(I2C_TypeDef* I2Cx, uint8_t SlaveAddr, uint16_t ReadAddr,
    uint8_t AddrLen, uint8_t *pBuffer, uint8_t NumBytesToRead)
{
    bool Result = false;

    if (xSemaphoreTake(xMutex, 5000) != pdTRUE) {
        return false;
    }

    if (!pBuffer || !NumBytesToRead) {
        xSemaphoreGive(xMutex);
        return false;
    }

    struct IfaceStruct *Iface = GetIface(I2Cx);
    if (!Iface) {
        xSemaphoreGive(xMutex);
        return false;
    }

    ResetIface(Iface);

    Iface->Addr.Val = ReadAddr;
    Iface->Addr.Len = AddrLen;

    Iface->Rx.Buf = pBuffer;
    Iface->Rx.Len = NumBytesToRead;

    if (!I2Cx_StartTransmission(I2Cx, I2C_TRANSMITTER, SlaveAddr, AddrLen ? AddrLen : NumBytesToRead)) {
        xSemaphoreGive(xMutex);
        return false;
    }

    /* Block until end of transaction */
    if (xSemaphoreTake(xSemphr, I2C_TRANSFER_TIMEOUT_MS(AddrLen + NumBytesToRead)) == pdTRUE) {
        Result = (Iface->Rx.Idx == Iface->Rx.Len);
    } else {
        if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
            /* STOP is sent automatically if NACK was received */
            LL_I2C_ClearFlag_NACK(I2Cx);
        }
    }

    xSemaphoreGive(xMutex);
    return Result;
}


bool I2Cx_WriteData(I2C_TypeDef* I2Cx, uint8_t SlaveAddr, uint16_t WriteAddr,
    uint8_t AddrLen, uint8_t *pBuffer, uint8_t NumBytesToWrite)
{
    bool Result = false;

    if (xSemaphoreTake(xMutex, 5000) != pdTRUE) {
        return false;
    }

    if (!pBuffer || !NumBytesToWrite) {
        xSemaphoreGive(xMutex);
        return false;
    }

    struct IfaceStruct *Iface = GetIface(I2Cx);
    if (!Iface) {
        xSemaphoreGive(xMutex);
        return false;
    }

    ResetIface(Iface);

    Iface->Addr.Val = WriteAddr;
    Iface->Addr.Len = AddrLen;

    Iface->Tx.Buf = pBuffer;
    Iface->Tx.Len = NumBytesToWrite;

    if (!I2Cx_StartTransmission(I2Cx, I2C_TRANSMITTER, SlaveAddr, AddrLen + NumBytesToWrite)) {
        xSemaphoreGive(xMutex);
        return false;
    }

    /* Block until end of transaction */
    if (xSemaphoreTake(xSemphr, I2C_TRANSFER_TIMEOUT_MS(AddrLen + NumBytesToWrite)) == pdTRUE) {
        Result = (Iface->Tx.Idx == Iface->Tx.Len);
    } else {
        if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
            /* STOP is sent automatically if NACK was received */
            LL_I2C_ClearFlag_NACK(I2Cx);
        }
    }

    xSemaphoreGive(xMutex);
    return Result;
}


static i2c_state_t I2Cx_SendByte_Callback(I2C_TypeDef* I2Cx)
{
    struct IfaceStruct *Iface = GetIface(I2Cx);

    if (!Iface) {
        return FAILED;
    }

    if (Iface->Addr.Len && (Iface->Addr.Idx < Iface->Addr.Len)) {
        if (Iface->Addr.Len == 2 && Iface->Addr.Idx == 0) {
            /* MSB */
            LL_I2C_TransmitData8(I2Cx, Iface->Addr.Val >> 8);
        } else {
            /* LSB */
            LL_I2C_TransmitData8(I2Cx, Iface->Addr.Val & 0xFF);
        }
        /* Address sent, disable TXE and wait TC */
        if (++Iface->Addr.Idx == Iface->Addr.Len) {
            /* Generate Restart only ro read requests */
            if (Iface->Rx.Len) {
                /* Address sent, disable TXE and wait TC */
                LL_I2C_DisableIT_TX(I2Cx);
                return RESTART;
            }
        }
    } else if (Iface->Tx.Len && (Iface->Tx.Idx < Iface->Tx.Len)) {
        LL_I2C_TransmitData8(I2Cx, Iface->Tx.Buf[Iface->Tx.Idx++]);
        if (Iface->Tx.Idx == Iface->Tx.Len) {
            /* All data sent, disable TXE and wait TC */
            LL_I2C_DisableIT_TX(I2Cx);
            return FINISHED;
        }
    } else {
        LL_I2C_ClearFlag_TXE(I2Cx);
    }
    return CONTINUE;
}


static void I2Cx_ReceiveByte_Callback(I2C_TypeDef* I2Cx, uint8_t byte)
{
    struct IfaceStruct *Iface = GetIface(I2Cx);

    if (!Iface) {
        return;
    }

    Iface->Rx.Buf[Iface->Rx.Idx++] = byte;
    if (Iface->Rx.Idx == Iface->Rx.Len - 1) {
        /* Send STOP after the last byte */
        LL_I2C_GenerateStopCondition(I2Cx);
    } else if (Iface->Rx.Idx == Iface->Rx.Len) {
        /* All data received.
         * Send STOP explicitly for a sigle byte transactions */
        I2Cx_StopTransmissionISR(I2Cx, (Iface->Rx.Len == 1));
    }
}


static void EV_Handler(I2C_TypeDef* I2Cx)
{
    static i2c_state_t State = FAILED;
    struct IfaceStruct *Iface = GetIface(I2Cx);

    if (!Iface) {
        return;
    }

    /* TXE set, TC cleared */
    if (LL_I2C_IsActiveFlag_TXE(I2Cx) && !LL_I2C_IsActiveFlag_TC(I2Cx)) {
        State = I2Cx_SendByte_Callback(I2Cx);
    }
    /* TC set */
    if (LL_I2C_IsActiveFlag_TC(I2Cx)) {
        if (State == RESTART) {
            Iface->Direction = Iface->Tx.Len > 0 ?
                I2C_TRANSMITTER : I2C_RECEIVER;
            
            LL_I2C_DisableIT_TX(I2Cx);
            LL_I2C_EnableIT_RX(I2Cx);

            LL_I2C_HandleTransfer(I2Cx, Iface->SlaveAddr, LL_I2C_ADDRSLAVE_7BIT,
                Iface->Rx.Len, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
            
            State = CONTINUE;
        } else if (State == FINISHED) {
            I2Cx_StopTransmissionISR(I2Cx, true);
        }
    }
    /* RXNE set */
    if (LL_I2C_IsActiveFlag_RXNE(I2Cx)) {
        I2Cx_ReceiveByte_Callback(I2Cx, LL_I2C_ReceiveData8(I2Cx));
    }
}


static void ER_Handler(I2C_TypeDef* I2Cx)
{
    struct IfaceStruct *Iface = GetIface(I2Cx);

    if (!Iface) {
        return;
    }

    /* I2C Bus error */
    if (LL_I2C_IsActiveFlag_BERR(I2Cx)) {
        /* Clear BERR flag */
        LL_I2C_ClearFlag_BERR(I2Cx);
    }

    /* I2C Arbitration Lost */
    if (LL_I2C_IsActiveFlag_ARLO(I2Cx)) {
        /* Clear ARLO flag */
        LL_I2C_ClearFlag_ARLO(I2Cx);
    }

    /* I2C Acknowledge failure */
    if (LL_I2C_IsActiveFlag_NACK(I2Cx)) {
        /* Special case: NACK was received with the last byte */
        if (Iface->Tx.Len && (Iface->Tx.Idx == Iface->Tx.Len)) {
            /* Set `Iface->Tx.Idx != Iface->Tx.Len` in order
             * to fail transaction result */
            Iface->Tx.Idx--;
        }
        /* STOP is sent automatically if NACK was received */
        I2Cx_StopTransmissionISR(I2Cx, false);
        /* Clear AF flag */
        LL_I2C_ClearFlag_NACK(I2Cx);
    }

    /* I2C Over-Run/Under-Run */
    if (LL_I2C_IsActiveFlag_OVR(I2Cx)) {
        /* Clear OVR flag */
        LL_I2C_ClearFlag_OVR(I2Cx);
    }
}

/* EV handlers */

void I2C1_IRQHandler(void)
{
    EV_Handler(I2C1);
    ER_Handler(I2C1);
}


void I2C2_IRQHandler(void)
{
    EV_Handler(I2C2);
    ER_Handler(I2C2);
}