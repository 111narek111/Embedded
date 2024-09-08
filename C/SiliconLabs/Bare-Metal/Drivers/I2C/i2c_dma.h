/***************************************************************************//**
 * @file
 * @brief Top level application functions for I2C driver
 *******************************************************************************
 * Description
 *
 * This file contains the top-level functions for configuring and controlling
 * the Inter-Integrated Circuit (I2C) communication on Silicon Labs devices. It
 * provides initialization routines, read and write functions, and other utility
 * functions to manage I2C transactions and handle data communication between
 * master and slave devices.
 *
 * Author
 * Narek Hakobyan
 ******************************************************************************/
#ifndef I2C_DMA_H_
#define I2C_DMA_H_
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_ldma.h"
#include "em_cmu.h"
#include <stdbool.h>

typedef enum {
    CALLBACLISCALLED = 0, /**< I2C non-blocking transfer has ended successfully and callBack is called. */
    WRONGSLAVEADDRESS = 1, /**< Slave address is wrong. */
    WRONGREGISTERADDRESS = 2, /**< Wrong data in transmit buffer. */
    I2CARBITRATIONLOST = 3, /**< Expecting sda value is different from the value to
     be send on the bus(without pull-up resistor or trace is cut) */
    I2CIDLEBUSTIMEOUT = 4, /**< During the transmission bus has gone idle. */
    I2CCLOCKLOWTIMEOUT = 5, /**< Clock is low too long. */
    I2CBUSERROR = 6, /**< Occurs when Start/stop is misplaced(SDA changes
     while SCL is high). */
    I2CCLOCKLOWSTARTSTOP = 7, /**< Device send start/stop while the other one
     tried to send data. */
    I2CSCLERROR = 8, /**< SDA is not high when trying to send start/stop. */
    I2CSDAERROR = 9, /**< SCL is not high when trying to send start/stop or at any clock cycle. */
    I2CTRANSFERISSTARTED = 10, /**< I2C Transfer is started. */
    I2CWRONGPORT = 11, /**< Wrong I2C port. */
    LDMANOFREECHANNEL = 12, /**< There is no free LDMA channel. */
    LDMAONLY1CHANNEL = 13, /**< There is only 1 free LDMA channel(2 required). */
    I2CBUSISBUSY = 14, /**< I2C is processing a transfer. */
    I2CTRANSFEREND = 15, /**< I2C blocking Transfer has ended. */
    UNKNOWNSTATE = 16, /**< unknown error. */
} I2C_NB_RETURN_T;

typedef struct {
    I2C_TypeDef *i2c; /**< Peripheral port */
    GPIO_Port_TypeDef SCLport; /**< SCL pin port number */
    uint8_t SCLpin; /**< SCL pin number */
    GPIO_Port_TypeDef SDAport; /**< SDA pin port number */
    uint8_t SDApin; /**< SDA pin number */
    bool master; /**< I2C mode */
    uint32_t refFreq; /**< I2C reference clock */
    uint32_t freq; /**< I2C max bus frequency to use */
    I2C_ClockHLR_TypeDef clhr; /**< Clock low/high ratio control */
    LDMA_Init_t ldmaInit; /**< Ldma Initialization struct */
} I2C_NB_INIT_T;

typedef struct {
    I2C_TypeDef *i2c; /**< Peripheral port */
    uint8_t slaveAddress; /**< Slave address to communicate */
    uint8_t *txBuff; /**< Buffer of register and commands to send to slave */
    uint8_t txCount; /**< Length of txBuff */
    uint8_t *rxBuff; /**< Buffer to receive data from slave */
    uint8_t rxCount; /**< Length of rxBuff */
    void (*callBack)(I2C_NB_RETURN_T); /**< Call-back function */
} I2C_NB_READ_T;

typedef struct {
    I2C_TypeDef *i2c; /**< Peripheral port */
    uint8_t slaveAddress; /**< Slave address to communicate */
    uint8_t *txBuff; /**< Buffer of register and commands to send to slave */
    uint8_t txCount; /**< Length of txBuff */
    void (*callBack)(I2C_NB_RETURN_T); /**< Call-back function */
} I2C_NB_WRITE_T;

typedef enum {
    I2CINITWRONGPORT = 0, /**< Wrong i2c. */
    I2CISENABLED = 1, /**< Peripheral is enabled. */
} I2C_NB_INIT_RETURN_T;




typedef void (*I2C_CALLFUNCTION_T)(I2C_NB_RETURN_T status);
typedef void (*I2C_CALLBACK_T)(uint32_t i2c_status, I2C_CALLFUNCTION_T callFunc);

/*******************************************************************************
 * @brief
 *   Initialize I2C peripheral for non-blocking transfer.
 *
 * @param[in] i2cNBInit
 *  Struct to initialize i2c for non-blocking transfers.
 *
 * @return
 *  Function state
 *
 * @note
 *  SDA and SCL monitoring is not enabled.
 *******************************************************************************/

I2C_NB_INIT_RETURN_T I2C_DMA_Init(I2C_NB_INIT_T *i2cNBInit);

/*******************************************************************************
 * @brief
 *  Start non-blocking i2c read transfer.
 *
 * @param[in] i2cNbReadInit
 *  Struct to start i2c non-blocking read.
 *
 * @return
 *  Function State
 *
 * @note
 *  Use this function when expecting data from device.
 *******************************************************************************/

I2C_NB_RETURN_T I2C_DMA_Read(I2C_NB_READ_T *i2cNbReadInit);

/*******************************************************************************
 * @brief
 *  Start non-blocking i2c write transfer.
 *
 * @param[in] i2cNbWriteInit
 *  Struct to start i2c non-blocking write.
 *
 * @return
 *  Function state
 *
 * @note
 *  Use this function when there is no need to read data from device.
 *******************************************************************************/

I2C_NB_RETURN_T I2C_DMA_Write(I2C_NB_WRITE_T *i2cNbWriteInit);

#endif  // I2C_DMA_H_
