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
#include "usart_dma.h"
#include "em_timer.h"
#include "em_cmu.h"
#include "em_ldma.h"

#define USART_DRV_INTERRUPTS                USART_IEN_TCMP1 | USART_IEN_TXC | USART_IEN_FERR | USART_IEN_PERR | USART_IEN_CCF

#define USART_DRV_TSTART()                  (USART_DRV_TSTART_RXEOF << USART_DRV_TSTART_SHIFT)
#define USART_DRV_TSTOP()                   (USART_DRV_TSTOP_RXACT << USART_DRV_TSTOP_SHIFT)
#define USART_DRV_USARTTIMER_CFG()          (\
                                               USART_DRV_TSTART() |\
                                               USART_DRV_TSTOP() |\
                                               USART_DRV_TCMPVAL_MAX\
                                             )

LDMA_Descriptor_t UsartDescTx[2];
LDMA_Descriptor_t UsartDescRx[2];
USART_DRV_TX_CALLBACK_T CallFunctionTx;
USART_DRV_RX_CALLBACK_T CallFunctionRx;
USART_TypeDef *Usart_rx;
uint8_t ChannelLdmaTx;
uint8_t ChannelLdmaRx;
uint8_t RxCount;

/*******************************************************************************
 * @brief
 *  USART0 receive interrupt handler.
 *******************************************************************************/

void USART0_RX_IRQHandler(void)
{
    TIMER_Enable(TIMER0, false);
    TIMER0->CNT_SET = USART_DRV_TIMER_DEFAULT;
    TIMER_Enable(TIMER0, true);

    USART_IntClear(USART0, USART_IntGet(USART0));
}

/*******************************************************************************
 * @brief
 *  USART0 transmit interrupt handler.
 *******************************************************************************/

void USART0_TX_IRQHandler(void)
{
    LDMA_StopTransfer(ChannelLdmaTx);
    LDMA->CHDONE_CLR = ChannelLdmaTx;
    USART0->CMD |= USART_CMD_CLEARTX;
    USART0->CMD |= USART_CMD_TXDIS;
    uint32_t usart_if = USART0->IF;
    USART_DRV_STATUS_E statusUsart;

    switch (usart_if & USART_DRV_ERR_MASK) {
    case USART_IF_CCF:
        statusUsart = USART_DRV_COLLISION_CHECK_ERR;
        break;
    case USART_IF_FERR:
        statusUsart = USART_DRV_FRAMING_ERR;
        break;
    case USART_IF_PERR:
        statusUsart = USART_DRV_PARITY_ERR;
        break;
    default:
        statusUsart = USART_DRV_STATUS_OK;
    }

    CallFunctionTx(statusUsart);

    USART_IntClear(USART0, USART_IntGet(USART0));
}

/*******************************************************************************
 * @brief
 *  USART1 receive interrupt handler.
 *******************************************************************************/

void USART1_RX_IRQHandler(void)
{
    TIMER_Enable(TIMER0, false);
    TIMER0->CNT_SET = USART_DRV_TIMER_DEFAULT;
    TIMER_Enable(TIMER0, true);

    USART_IntClear(USART1, USART_IntGet(USART1));
}

/*******************************************************************************
 * @brief
 *  USART1 transmit interrupt handler.
 *******************************************************************************/

void USART1_TX_IRQHandler(void)
{
    LDMA_StopTransfer(ChannelLdmaTx);
    LDMA->CHDONE_CLR = ChannelLdmaTx;
    USART1->CMD |= USART_CMD_CLEARTX;
    USART1->CMD |= USART_CMD_TXDIS;
    uint32_t usart_if = USART1->IF;
    USART_DRV_STATUS_E statusUsart;

    switch (usart_if & USART_DRV_ERR_MASK) {
    case USART_IF_CCF:
        statusUsart = USART_DRV_COLLISION_CHECK_ERR;
        break;
    case USART_IF_FERR:
        statusUsart = USART_DRV_FRAMING_ERR;
        break;
    case USART_IF_PERR:
        statusUsart = USART_DRV_PARITY_ERR;
        break;
    default:
        statusUsart = USART_DRV_STATUS_OK;
    }

    CallFunctionTx(statusUsart);

    USART_IntClear(USART1, USART_IntGet(USART1));
}

/*******************************************************************************
 * @brief
 *  TIMER0 interrupt handler.
 *******************************************************************************/

void TIMER0_IRQHandler(void)
{

    LDMA_StopTransfer(ChannelLdmaRx);
    RxCount = USART_DRV_RX_MAX_BUFF_SIZE - LDMA_TransferRemainingCount(ChannelLdmaRx);
    LDMA->CHDONE_CLR = ChannelLdmaTx;
    Usart_rx->CMD |= USART_CMD_CLEARRX;
    Usart_rx->CMD |= USART_CMD_RXDIS;
    uint32_t usart_if = Usart_rx->IF;
    USART_DRV_STATUS_E statusUsart;

    switch (usart_if & USART_DRV_ERR_MASK) {
    case USART_IF_CCF:
        statusUsart = USART_DRV_COLLISION_CHECK_ERR;
        break;
    case USART_IF_FERR:
        statusUsart = USART_DRV_FRAMING_ERR;
        break;
    case USART_IF_PERR:
        statusUsart = USART_DRV_PARITY_ERR;
        break;
    default:
        statusUsart = USART_DRV_STATUS_OK;
    }

    CallFunctionRx(statusUsart, RxCount);

    TIMER_IntClear(TIMER0, TIMER_IF_OF);
}

/*******************************************************************
 * @brief
 *  Find free ldma channel and return it.
 *
 * @return
 *  Free ldma channel.
 *******************************************************************/

static uint8_t LDMA_GetFreeChannel(void)
{
    uint32_t chStatus = LDMA->CHSTATUS;
    uint8_t i = 0;
    uint8_t ch = 0;
    while (i < 8)
    {
        if (chStatus & 1)
                {
            chStatus = chStatus >> 1;
            ch++;
            i++;
        }
        else
        {
            return ch;
        }
    }
    return 255;
}

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

USART_DRV_RETURN_E usart_drv_transmit(USART_TypeDef *usart, uint8_t *txBuf, uint8_t size, USART_DRV_TX_CALLBACK_T callBack)
{

    if ((usart->STATUS & USART_DRV_TX_BUSY_MASK) != 0) {
        return USART_DRV_USART_BUSY;
    }

    ChannelLdmaTx = LDMA_GetFreeChannel();
    uint8_t chmask;
    if (ChannelLdmaTx != USART_DRV_LDMA_NO_CHANNEL) {
        chmask = 1 << ChannelLdmaTx;
    }
    else {
        return USART_DRV_LDMA_NOFREECHANNEL;
    }

    uint32_t txenable = USART_CMD_TXEN;

    UsartDescTx[0].xfer.srcAddr = (uint32_t) &txenable;
    UsartDescTx[0].xfer.dstAddr = (uint32_t) &usart->CMD;

    UsartDescTx[1].xfer.srcAddr = (uint32_t) txBuf;
    UsartDescTx[1].xfer.dstAddr = (uint32_t) &usart->TXDATA;
    UsartDescTx[1].xfer.xferCnt = size - 1;

    LDMA_PeripheralSignal_t signal;
    if (usart == USART0) {
        signal = ldmaPeripheralSignal_USART0_TXBL;
    }
    else if (usart == USART1) {
        signal = ldmaPeripheralSignal_USART1_TXBL;
    }
    else {
        return USART_DRV_WRONGUSART;
    }

    CallFunctionTx = callBack;

    usart->CMD |= USART_CMD_CLEARTX;

    LDMAXBAR->CH[ChannelLdmaTx].REQSEL = signal;
    LDMA->CH[ChannelLdmaTx].LINK = (uint32_t) &UsartDescTx & _LDMA_CH_LINK_LINKADDR_MASK;
    LDMA->LINKLOAD |= chmask;

    return USART_DRV_FUNCTION_DONE;
}

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

USART_DRV_RETURN_E usart_drv_receive(USART_TypeDef *usart, uint8_t *rxBuf, USART_DRV_RX_CALLBACK_T callBack)
{
    if ((usart->STATUS & USART_DRV_RX_BUSY_MASK) != 0) {
        if ((TIMER0->STATUS & TIMER_STATUS_RUNNING) != 0) {
            return USART_DRV_USART_BUSY;
        }
    }

    ChannelLdmaRx = LDMA_GetFreeChannel();
    uint8_t chmask;
    if (ChannelLdmaRx != USART_DRV_LDMA_NO_CHANNEL) {
        chmask = 1 << ChannelLdmaRx;
    }
    else {
        return USART_DRV_LDMA_NOFREECHANNEL;
    }
    Usart_rx = usart;

    uint32_t rxenable = USART_CMD_RXEN;

    UsartDescRx[0].xfer.srcAddr = (uint32_t) &rxenable;
    UsartDescRx[0].xfer.dstAddr = (uint32_t) &usart->CMD;

    UsartDescRx[1].xfer.srcAddr = (uint32_t) &usart->RXDATA;
    UsartDescRx[1].xfer.dstAddr = (uint32_t) rxBuf;
    UsartDescRx[1].xfer.xferCnt = (USART_DRV_RX_MAX_BUFF_SIZE - 1);

    LDMA_PeripheralSignal_t signal;
    if (usart == USART0) {
        signal = ldmaPeripheralSignal_USART0_RXDATAV;
    }
    else if (usart == USART1) {
        signal = ldmaPeripheralSignal_USART1_RXDATAV;
    }
    else {
        return USART_DRV_WRONGUSART;
    }

    CallFunctionRx = callBack;

    usart->CMD |= USART_CMD_CLEARRX;

    LDMAXBAR->CH[ChannelLdmaRx].REQSEL = signal;
    LDMA->CH[ChannelLdmaRx].LINK = (uint32_t) &UsartDescRx & _LDMA_CH_LINK_LINKADDR_MASK;
    LDMA->LINKLOAD |= chmask;

    return USART_DRV_FUNCTION_DONE;
}

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

USART_DRV_RETURN_E usart_drv_start(USART_TypeDef *usart)
{
    IRQn_Type irqTx, irqRx;
    if (usart == USART0) {
        irqTx = USART0_TX_IRQn;
        irqRx = USART0_RX_IRQn;
    }
    else if (usart == USART1) {
        irqTx = USART1_TX_IRQn;
        irqRx = USART1_RX_IRQn;
    }
    else {
        return USART_DRV_WRONGUSART;
    }

    TIMER_IntEnable(TIMER0, TIMER_IEN_OF);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER0_IRQn);

    usart->EN = USART_DRV_ENABLE;
    USART_IntEnable(usart, USART_DRV_INTERRUPT_MASK);

    NVIC_ClearPendingIRQ(irqTx);
    NVIC_ClearPendingIRQ(irqRx);
    NVIC_EnableIRQ(irqTx);
    NVIC_EnableIRQ(irqRx);

    return USART_DRV_FUNCTION_DONE;
}

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

USART_DRV_RETURN_E usart_drv_stop(USART_TypeDef *usart)
{
    IRQn_Type irqTx, irqRx;
    if (usart == USART0) {
        irqTx = USART0_TX_IRQn;
        irqRx = USART0_RX_IRQn;
    }
    else if (usart == USART1) {
        irqTx = USART1_TX_IRQn;
        irqRx = USART1_RX_IRQn;
    }
    else {
        return USART_DRV_WRONGUSART;
    }

    TIMER_IntDisable(TIMER0, TIMER_IEN_OF);
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
    NVIC_DisableIRQ(TIMER0_IRQn);

    usart->EN = USART_DRV_DISABLE;
    USART_IntDisable(usart, USART_DRV_INTERRUPT_MASK);

    NVIC_ClearPendingIRQ(irqTx);
    NVIC_ClearPendingIRQ(irqRx);
    NVIC_DisableIRQ(irqTx);
    NVIC_DisableIRQ(irqRx);

    return USART_DRV_FUNCTION_DONE;
}

/*******************************************************************************
 * @brief
 *  Initialize non-blocking UART.
 *
 * @param[in] usart
 *  USART peripheral port.
 *******************************************************************************/

void usart_drv_init(USART_TypeDef *usart)
{
    UsartDescTx[0].xfer.structType = ldmaCtrlStructTypeXfer;
    UsartDescTx[0].xfer.structReq = 0;
    UsartDescTx[0].xfer.byteSwap = 0;
    UsartDescTx[0].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    UsartDescTx[0].xfer.doneIfs = 0;
    UsartDescTx[0].xfer.reqMode = ldmaCtrlReqModeBlock;
    UsartDescTx[0].xfer.decLoopCnt = 0;
    UsartDescTx[0].xfer.ignoreSrec = 0;
    UsartDescTx[0].xfer.srcInc = ldmaCtrlSrcIncOne;
    UsartDescTx[0].xfer.size = ldmaCtrlSizeByte;
    UsartDescTx[0].xfer.dstInc = ldmaCtrlDstIncNone;
    UsartDescTx[0].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    UsartDescTx[0].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    UsartDescTx[0].xfer.linkMode = ldmaLinkModeRel;
    UsartDescTx[0].xfer.link = 1;
    UsartDescTx[0].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;
    UsartDescTx[0].xfer.xferCnt = 0;

    UsartDescTx[1].xfer.structType = ldmaCtrlStructTypeXfer;
    UsartDescTx[1].xfer.structReq = 0;
    UsartDescTx[1].xfer.byteSwap = 0;
    UsartDescTx[1].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    UsartDescTx[1].xfer.doneIfs = 0;
    UsartDescTx[1].xfer.reqMode = ldmaCtrlReqModeBlock;
    UsartDescTx[1].xfer.decLoopCnt = 0;
    UsartDescTx[1].xfer.ignoreSrec = 0;
    UsartDescTx[1].xfer.srcInc = ldmaCtrlSrcIncOne;
    UsartDescTx[1].xfer.size = ldmaCtrlSizeByte;
    UsartDescTx[1].xfer.dstInc = ldmaCtrlDstIncNone;
    UsartDescTx[1].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    UsartDescTx[1].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    UsartDescTx[1].xfer.linkMode = 0;
    UsartDescTx[1].xfer.link = 0;
    UsartDescTx[1].xfer.linkAddr = 0;

    UsartDescRx[0].xfer.structType = ldmaCtrlStructTypeXfer;
    UsartDescRx[0].xfer.structReq = 1;
    UsartDescRx[0].xfer.byteSwap = 0;
    UsartDescRx[0].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    UsartDescRx[0].xfer.doneIfs = 0;
    UsartDescRx[0].xfer.reqMode = ldmaCtrlReqModeBlock;
    UsartDescRx[0].xfer.decLoopCnt = 0;
    UsartDescRx[0].xfer.ignoreSrec = 0;
    UsartDescRx[0].xfer.srcInc = ldmaCtrlSrcIncOne;
    UsartDescRx[0].xfer.size = ldmaCtrlSizeByte;
    UsartDescRx[0].xfer.dstInc = ldmaCtrlDstIncNone;
    UsartDescRx[0].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    UsartDescRx[0].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    UsartDescRx[0].xfer.linkMode = ldmaLinkModeRel;
    UsartDescRx[0].xfer.link = 1;
    UsartDescRx[0].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;
    UsartDescRx[0].xfer.xferCnt = 0;

    UsartDescRx[1].xfer.structType = ldmaCtrlStructTypeXfer;
    UsartDescRx[1].xfer.structReq = 0;
    UsartDescRx[1].xfer.byteSwap = 0;
    UsartDescRx[1].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    UsartDescRx[1].xfer.doneIfs = 0;
    UsartDescRx[1].xfer.reqMode = ldmaCtrlReqModeBlock;
    UsartDescRx[1].xfer.decLoopCnt = 0;
    UsartDescRx[1].xfer.ignoreSrec = 0;
    UsartDescRx[1].xfer.srcInc = ldmaCtrlSrcIncNone;
    UsartDescRx[1].xfer.size = ldmaCtrlSizeByte;
    UsartDescRx[1].xfer.dstInc = ldmaCtrlDstIncOne;
    UsartDescRx[1].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    UsartDescRx[1].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    UsartDescRx[1].xfer.linkMode = 0;
    UsartDescRx[1].xfer.link = 0;
    UsartDescRx[1].xfer.linkAddr = 0;

    LDMA_Init_t ldmaInit = LDMA_INIT_DEFAULT;
    LDMA_Init(&ldmaInit);

    uint32_t timerTop = CMU_ClockFreqGet(cmuClock_TIMER0);
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.enable = false;
    timerInit.oneShot = true;
    TIMER_Init(TIMER0, &timerInit);
    timerTop = timerTop * USART_DRV_RX_TIMEOUT;
    TIMER_TopSet(TIMER0, timerTop);

    usart->TIMECMP1 = USART_DRV_USARTTIMER_CFG();
    USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;
    usartInit.enable = usartDisable;
    USART_InitAsync(usart, &usartInit);
    usart->CTRL |= (USART_CTRL_ERRSTX | USART_CTRL_ERRSRX);

    usart->EN = USART_DRV_DISABLE;
}
