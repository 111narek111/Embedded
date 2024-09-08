/***************************************************************************//**
 * @file
 * @brief Top level application functions for USART driver
 *******************************************************************************
 * Description
 *
 * This file contains the top-level functions for configuring and controlling
 * the Universal Synchronous/Asynchronous Receiver/Transmitter (USART) on
 * Silicon Labs devices. It provides initialization routines, functions for
 * transmitting and receiving data, and other utility functions to manage
 * serial communication in asynchronous modes.
 *
 * Author
 * Narek Hakobyan
 ******************************************************************************/
#ifndef USART_DMA_H_
#define USART_DMA_H_

#include "em_usart.h"

#define USART_DRV_TSTART_RXEOF             4
#define USART_DRV_TSTART_SHIFT             16
#define USART_DRV_TSTOP_RXACT              2
#define USART_DRV_TSTOP_SHIFT              20
#define USART_DRV_TCMPVAL_MAX              0xFF  /*256 baud times */
#define USART_DRV_TIMER_RESTARTEN          1
#define USART_DRV_TIMER_RESTARTEN_SHIFT    24
#define USART_DRV_RX_MAX_BUFF_SIZE         100
#define USART_DRV_TIMER_DEFAULT            0
#define USART_DRV_LDMA_NO_CHANNEL          0xFF
#define USART_DRV_RX_TIMEOUT               0.02 //20 miliseconds
#define USART_DRV_ERR_MASK                 0x1300
#define USART_DRV_DISABLE                  0
#define USART_DRV_ENABLE                   1
#define USART_DRV_TX_BUSY_MASK             0x30000
#define USART_DRV_RX_BUSY_MASK             0x40
#define USART_DRV_INTERRUPT_MASK           0x9301

typedef enum {
    USART_DRV_STATUS_OK = 0,
    USART_DRV_PARITY_ERR,
    USART_DRV_FRAMING_ERR,
    USART_DRV_COLLISION_CHECK_ERR
}USART_DRV_STATUS_E;


typedef enum {
    USART_DRV_FUNCTION_DONE = 0,
    USART_DRV_WRONGUSART,
    USART_DRV_LDMA_NOFREECHANNEL,
    USART_DRV_USART_BUSY
}USART_DRV_RETURN_E;

typedef void (*USART_DRV_RX_CALLBACK_T)(USART_DRV_STATUS_E uartStatus,uint8_t rxSize);
typedef void (*USART_DRV_TX_CALLBACK_T)(USART_DRV_STATUS_E uartStatus);

typedef void (*PSP_DMA_DONE)(uint32_t result, uint16_t Cnt);


/*******************************************************************************
 * @brief
 *  Initialize non-blocking UART.
 *
 * @param[in] usart
 *  USART peripheral port.
 *******************************************************************************/

void usart_drv_init(USART_TypeDef *usart);

/*******************************************************************************
 * @brief
 *  Start work of non-blocking UART.
 *
 * @param[in] usart
 *  USART peripheral port.
 *
 * @return
 *  Function state
 *******************************************************************************/

USART_DRV_RETURN_E usart_drv_start(USART_TypeDef *usart);

/*******************************************************************************
 * @brief
 *  Stop work of non-blocking UART.
 *
 * @param[in] usart
 *  USART peripheral port.
 *
 * @return
 *  Function state
 *******************************************************************************/

USART_DRV_RETURN_E usart_drv_stop(USART_TypeDef *usart);

/*******************************************************************************
 * @brief
 *  Transmit data with USART.
 *
 * @param[in] usart
 *  USART peripheral port.
 *
 * @param[in] txBuf
 *  Buffer for data that will be transmitted.
 *
 * @param[in] size
 *  Size of txBuf.
 *
 * @param[in] callBack
 *  Function that will be called when transfer will be done.
 *
 * @return
 *  Function state
 *******************************************************************************/

USART_DRV_RETURN_E usart_drv_transmit(USART_TypeDef *usart,uint8_t *txBuf,uint8_t size,USART_DRV_TX_CALLBACK_T callBack);

/*******************************************************************************
 * @brief
 *  Receive data with USART.
 *
 * @param[in] usart
 *  USART peripheral port.
 *
 * @param[in] rxBuf
 *  Buffer for received data.
 *
 * @param[in] callBack
 *  Function that will be called when transfer will be done.
 *
 * @return
 *  Function state
 *******************************************************************************/

USART_DRV_RETURN_E usart_drv_receive(USART_TypeDef *usart,uint8_t *rxBuf,USART_DRV_RX_CALLBACK_T callBack);



#endif /* USART_DMA_H_ */
