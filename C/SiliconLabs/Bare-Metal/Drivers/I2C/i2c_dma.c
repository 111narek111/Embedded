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
#include "i2c_dma.h"
#include "ldma_desc.h"

#define I2C_ERR_MASK    0x1CC680LU; /**< I2C errors interrupt flag */

static void I2CNB_callBack(uint32_t i2cErrStat, I2C_CALLFUNCTION_T callFunc);

uint32_t chmaskTx, chmaskRx;
uint32_t i2cInterrupts = I2C_IEN_MSTOP | I2C_IEN_ARBLOST | I2C_IEN_BITO | I2C_IEN_BUSERR | I2C_IEN_CLTO | I2C_IEN_CLERR | I2C_IEN_SCLERR | I2C_IEN_SDAERR | I2C_IEN_NACK;
uint32_t i2c_Flags;
uint32_t i2cReqTx, i2cReqRx;
uint8_t sladdWr, sladdRd, last, prelast, txcount, rxcount, tcounter;
uint8_t i2cStart = I2C_CMD_START;
uint8_t i2cNack = I2C_CMD_NACK | I2C_CMD_STOP;
uint8_t autoA = I2C_CTRL_AUTOACK;
uint8_t txClear = I2C_CMD_CLEARTX;
uint8_t whichTrans = 0;
I2C_TypeDef *i2cChoice = I2C0;

I2C_CALLFUNCTION_T callFunc = 0;
I2C_CALLBACK_T callBack;

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
    return 10;
}

/*******************************************************************************
 * @brief
 *  I2C0 interrupt handler.
 *******************************************************************************/

void I2C0_IRQHandler(void)
{
    LDMA->CHDIS = chmaskTx | chmaskRx;
    I2C0->CMD |= I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
    i2c_Flags = I2C0->IF;
    I2C_IntClear(I2C0, I2C_IntGet(I2C0));
    LDMA->CHDONE_CLR = chmaskTx | chmaskRx;
    I2C0->IEN_CLR = i2cInterrupts;
    I2C0->CTRL_CLR = I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSE;
    I2CNB_callBack(i2c_Flags, callFunc);
}

/*******************************************************************************
 * @brief
 *  I2C1 interrupt handler.
 *******************************************************************************/

void I2C1_IRQHandler(void)
{
    LDMA->CHDIS = chmaskTx | chmaskRx;
    I2C1->CMD |= I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
    i2c_Flags = I2C1->IF;
    I2C_IntClear(I2C1, I2C_IntGet(I2C1));
    LDMA->CHDONE_CLR = chmaskTx | chmaskRx;
    I2C1->IEN_CLR = i2cInterrupts;
    I2C1->CTRL_CLR = I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSE;
    I2CNB_callBack(i2c_Flags, callFunc);
}

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

I2C_NB_INIT_RETURN_T I2C_DMA_Init(I2C_NB_INIT_T *i2cNBInit)
{
    CMU_ClockEnable(cmuClock_LDMA, true);
    CMU_ClockEnable(cmuClock_GPIO, true);
    I2C_Init_TypeDef i2cInit;
    uint32_t i2cClock;
    uint8_t i2cNum;
    if (false) {
#if defined(I2C0)
    }
    else if (i2cNBInit->i2c == I2C0) {
        i2cClock = cmuClock_I2C0;
        i2cNum = 0;
#endif
#if defined(I2C1)
    }
    else if (i2cNBInit->i2c == I2C1) {
        i2cClock = cmuClock_I2C1;
        i2cNum = 1;
#endif
#if defined(I2C2)
  } else if (i2cInit->i2c == I2C2) {
    i2cClock = cmuClock_I2C2;
    i2cNum = 2;
#endif
    }
    else {
        /* I2C clock is not defined */
        EFM_ASSERT(false);
        return I2CINITWRONGPORT;
    }

    CMU_ClockEnable(i2cClock, true);

    //Pin Configuration
    GPIO_PinModeSet(i2cNBInit->SCLport, i2cNBInit->SCLpin, gpioModeWiredAndPullUp, 1);
    GPIO_PinModeSet(i2cNBInit->SDAport, i2cNBInit->SDApin, gpioModeWiredAndPullUp, 1);

    GPIO->I2CROUTE[i2cNum].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
    GPIO->I2CROUTE[i2cNum].SCLROUTE = (uint32_t) ((i2cNBInit->SCLpin << _GPIO_I2C_SCLROUTE_PIN_SHIFT)
            | (i2cNBInit->SCLport << _GPIO_I2C_SCLROUTE_PORT_SHIFT));
    GPIO->I2CROUTE[i2cNum].SDAROUTE = (uint32_t) ((i2cNBInit->SDApin << _GPIO_I2C_SDAROUTE_PIN_SHIFT)
            | (i2cNBInit->SDAport << _GPIO_I2C_SDAROUTE_PORT_SHIFT));

    i2cInit.enable = false;
    i2cInit.master = i2cNBInit->master; /* master mode only */
    i2cInit.freq = i2cNBInit->freq;
    i2cInit.refFreq = i2cNBInit->refFreq;
    i2cInit.clhr = i2cNBInit->clhr;

    I2C_Init(i2cNBInit->i2c, &i2cInit);
    i2cNBInit->i2c->CTRL |= I2C_CTRL_BITO_I2C160PCC | I2C_CTRL_CLTO_I2C1024PCC;

    LDMA_Init(&i2cNBInit->ldmaInit);

    //LDMA descriptor configurations that are constant

    descTxRead[0].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[0].xfer.size = ldmaCtrlSizeByte;
    descTxRead[0].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descTxRead[0].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descTxRead[0].xfer.structReq = 0;
    descTxRead[0].xfer.byteSwap = 0;
    descTxRead[0].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[0].xfer.doneIfs = 0;
    descTxRead[0].xfer.reqMode = ldmaCtrlReqModeBlock;
    descTxRead[0].xfer.decLoopCnt = 0;
    descTxRead[0].xfer.ignoreSrec = 0;

    descTxRead[0].xfer.srcInc = ldmaCtrlSrcIncOne;
    descTxRead[0].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxRead[0].xfer.xferCnt = 0;
    descTxRead[0].xfer.linkMode = ldmaLinkModeRel;
    descTxRead[0].xfer.link = 1;

    descTxRead[1].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[1].xfer.size = ldmaCtrlSizeByte;
    descTxRead[1].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descTxRead[1].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descTxRead[1].xfer.structReq = 0;
    descTxRead[1].xfer.byteSwap = 0;
    descTxRead[1].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[1].xfer.doneIfs = 0;
    descTxRead[1].xfer.reqMode = ldmaCtrlReqModeBlock;
    descTxRead[1].xfer.decLoopCnt = 0;
    descTxRead[1].xfer.ignoreSrec = 0;

    descTxRead[1].xfer.srcInc = ldmaCtrlSrcIncOne;
    descTxRead[1].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxRead[1].xfer.xferCnt = 0;
    descTxRead[1].xfer.linkMode = ldmaLinkModeRel;
    descTxRead[1].xfer.link = 1;
    descTxRead[1].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxRead[2].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[2].xfer.size = ldmaCtrlSizeByte,
    descTxRead[2].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxRead[2].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxRead[2].xfer.structReq = 0,
    descTxRead[2].xfer.byteSwap = 0,
    descTxRead[2].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[2].xfer.doneIfs = 0,
    descTxRead[2].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxRead[2].xfer.decLoopCnt = 0,
    descTxRead[2].xfer.ignoreSrec = 0,

    descTxRead[2].xfer.xferCnt = 0;
    descTxRead[2].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxRead[2].xfer.dstInc = ldmaCtrlDstIncOne;
    descTxRead[2].xfer.linkMode = ldmaLinkModeRel;
    descTxRead[2].xfer.link = 1;
    descTxRead[2].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxRead[3].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[3].xfer.structReq = 0;
    descTxRead[3].xfer.byteSwap = 0;
    descTxRead[3].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[3].xfer.doneIfs = 0;
    descTxRead[3].xfer.reqMode = ldmaCtrlReqModeBlock;
    descTxRead[3].xfer.decLoopCnt = 0;
    descTxRead[3].xfer.ignoreSrec = 0;
    descTxRead[3].xfer.size = ldmaCtrlSizeByte;
    descTxRead[3].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descTxRead[3].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;

    descTxRead[3].xfer.srcInc = ldmaCtrlSrcIncOne;
    descTxRead[3].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxRead[3].xfer.linkMode = ldmaLinkModeRel;
    descTxRead[3].xfer.link = 1;
    descTxRead[3].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxRead[4].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[4].xfer.size = ldmaCtrlSizeByte;
    descTxRead[4].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descTxRead[4].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descTxRead[4].xfer.structReq = 0;
    descTxRead[4].xfer.byteSwap = 0;
    descTxRead[4].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[4].xfer.doneIfs = 0;
    descTxRead[4].xfer.reqMode = ldmaCtrlReqModeBlock;
    descTxRead[4].xfer.decLoopCnt = 0;
    descTxRead[4].xfer.ignoreSrec = 0;

    descTxRead[4].xfer.srcInc = ldmaCtrlSrcIncOne;
    descTxRead[4].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxRead[4].xfer.xferCnt = 0;
    descTxRead[4].xfer.linkMode = ldmaLinkModeRel;
    descTxRead[4].xfer.link = 1;
    descTxRead[4].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxRead[5].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[5].xfer.size = ldmaCtrlSizeByte,
    descTxRead[5].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxRead[5].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxRead[5].xfer.structReq = 0,
    descTxRead[5].xfer.byteSwap = 0,
    descTxRead[5].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[5].xfer.doneIfs = 0,
    descTxRead[5].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxRead[5].xfer.decLoopCnt = 0,
    descTxRead[5].xfer.ignoreSrec = 0,

    descTxRead[5].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxRead[5].xfer.dstInc = ldmaCtrlDstIncOne;
    descTxRead[5].xfer.xferCnt = 0;
    descTxRead[5].xfer.linkMode = ldmaLinkModeRel;
    descTxRead[5].xfer.link = 1;
    descTxRead[5].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxRead[6].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[6].xfer.size = ldmaCtrlSizeByte;
    descTxRead[6].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descTxRead[6].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descTxRead[6].xfer.structReq = 0;
    descTxRead[6].xfer.byteSwap = 0;
    descTxRead[6].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[6].xfer.doneIfs = 0;
    descTxRead[6].xfer.reqMode = ldmaCtrlReqModeBlock;
    descTxRead[6].xfer.decLoopCnt = 0;
    descTxRead[6].xfer.ignoreSrec = 0;

    descTxRead[6].xfer.srcInc = ldmaCtrlSrcIncOne;
    descTxRead[6].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxRead[6].xfer.xferCnt = 0;
    descTxRead[6].xfer.linkMode = 0;
    descTxRead[6].xfer.link = 0;
    descTxRead[6].xfer.linkAddr = 0;

    descTxRead[7].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxRead[7].xfer.size = ldmaCtrlSizeByte;
    descTxRead[7].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descTxRead[7].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descTxRead[7].xfer.structReq = 0;
    descTxRead[7].xfer.byteSwap = 0;
    descTxRead[7].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxRead[7].xfer.doneIfs = 0;
    descTxRead[7].xfer.reqMode = ldmaCtrlReqModeBlock;
    descTxRead[7].xfer.decLoopCnt = 0;
    descTxRead[7].xfer.ignoreSrec = 0;

    descTxRead[7].xfer.srcInc = ldmaCtrlSrcIncOne;
    descTxRead[7].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxRead[7].xfer.xferCnt = 0;
    descTxRead[7].xfer.linkMode = 0;
    descTxRead[7].xfer.link = 0;
    descTxRead[7].xfer.linkAddr = 0;

    descRxRead[0].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[0].xfer.structReq = 0;
    descRxRead[0].xfer.byteSwap = 0;
    descRxRead[0].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[0].xfer.doneIfs = 0;
    descRxRead[0].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[0].xfer.decLoopCnt = 0;
    descRxRead[0].xfer.ignoreSrec = 0;
    descRxRead[0].xfer.size = ldmaCtrlSizeByte;
    descRxRead[0].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[0].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;

    descRxRead[0].xfer.srcInc = ldmaCtrlSrcIncOne;
    descRxRead[0].xfer.dstInc = ldmaCtrlDstIncNone;
    descRxRead[0].xfer.xferCnt = 0;
    descRxRead[0].xfer.linkMode = ldmaLinkModeRel;
    descRxRead[0].xfer.link = 1;

    descRxRead[1].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[1].xfer.structReq = 0;
    descRxRead[1].xfer.byteSwap = 0;
    descRxRead[1].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[1].xfer.doneIfs = 0;
    descRxRead[1].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[1].xfer.decLoopCnt = 0;
    descRxRead[1].xfer.ignoreSrec = 0;
    descRxRead[1].xfer.size = ldmaCtrlSizeByte;
    descRxRead[1].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[1].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;

    descRxRead[1].xfer.srcInc = ldmaCtrlSrcIncNone;
    descRxRead[1].xfer.dstInc = ldmaCtrlDstIncOne;
    descRxRead[1].xfer.linkMode = ldmaLinkModeRel;
    descRxRead[1].xfer.link = 1;
    descRxRead[1].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descRxRead[2].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[2].xfer.size = ldmaCtrlSizeByte;
    descRxRead[2].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[2].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[2].xfer.structReq = 0;
    descRxRead[2].xfer.byteSwap = 0;
    descRxRead[2].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[2].xfer.doneIfs = 0;
    descRxRead[2].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[2].xfer.decLoopCnt = 0;
    descRxRead[2].xfer.ignoreSrec = 0;

    descRxRead[2].xfer.srcInc = ldmaCtrlSrcIncOne;
    descRxRead[2].xfer.dstInc = ldmaCtrlDstIncNone;
    descRxRead[2].xfer.xferCnt = 0;
    descRxRead[2].xfer.linkMode = ldmaLinkModeRel;
    descRxRead[2].xfer.link = 1;

    descRxRead[3].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[3].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[3].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[3].xfer.structReq = 0;
    descRxRead[3].xfer.byteSwap = 0;
    descRxRead[3].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[3].xfer.doneIfs = 0;
    descRxRead[3].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[3].xfer.decLoopCnt = 0;
    descRxRead[3].xfer.ignoreSrec = 0;
    descRxRead[3].xfer.size = ldmaCtrlSizeByte;

    descRxRead[3].xfer.srcInc = ldmaCtrlSrcIncNone;
    descRxRead[3].xfer.xferCnt = 0;
    descRxRead[3].xfer.dstInc = ldmaCtrlDstIncOne;
    descRxRead[3].xfer.linkMode = ldmaLinkModeRel;
    descRxRead[3].xfer.link = 1;
    descRxRead[3].xfer.linkAddr = 2 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descRxRead[4].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[4].xfer.size = ldmaCtrlSizeByte;
    descRxRead[4].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[4].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[4].xfer.structReq = 0;
    descRxRead[4].xfer.byteSwap = 0;
    descRxRead[4].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[4].xfer.doneIfs = 0;
    descRxRead[4].xfer.reqMode = ldmaCtrlReqModeBlock,
    descRxRead[4].xfer.decLoopCnt = 0;
    descRxRead[4].xfer.ignoreSrec = 0;

    descRxRead[4].xfer.srcInc = ldmaCtrlSrcIncNone;
    descRxRead[4].xfer.dstInc = ldmaCtrlDstIncOne;
    descRxRead[4].xfer.xferCnt = 0;
    descRxRead[4].xfer.linkMode = ldmaLinkModeRel;
    descRxRead[4].xfer.link = 1;
    descRxRead[4].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descRxRead[5].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[5].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[5].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[5].xfer.structReq = 0;
    descRxRead[5].xfer.byteSwap = 0;
    descRxRead[5].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[5].xfer.doneIfs = 0;
    descRxRead[5].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[5].xfer.decLoopCnt = 0;
    descRxRead[5].xfer.ignoreSrec = 0;
    descRxRead[5].xfer.size = ldmaCtrlSizeByte;

    descRxRead[5].xfer.srcInc = ldmaCtrlSrcIncOne;
    descRxRead[5].xfer.dstInc = ldmaCtrlDstIncNone;
    descRxRead[5].xfer.xferCnt = 0;
    descRxRead[5].xfer.linkMode = ldmaLinkModeRel;
    descRxRead[5].xfer.link = 1;

    descRxRead[6].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[6].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[6].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[6].xfer.byteSwap = 0;
    descRxRead[6].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[6].xfer.doneIfs = 0;
    descRxRead[6].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[6].xfer.decLoopCnt = 0;
    descRxRead[6].xfer.ignoreSrec = 0;
    descRxRead[6].xfer.size = ldmaCtrlSizeByte;
    descRxRead[6].xfer.structReq = 0;

    descRxRead[6].xfer.srcInc = ldmaCtrlSrcIncNone;
    descRxRead[6].xfer.dstInc = ldmaCtrlDstIncOne;
    descRxRead[6].xfer.xferCnt = 0;
    descRxRead[6].xfer.linkMode = 0;
    descRxRead[6].xfer.link = 0;
    descRxRead[6].xfer.linkAddr = 0;

    descRxRead[7].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[7].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[7].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[7].xfer.byteSwap = 0;
    descRxRead[7].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[7].xfer.doneIfs = 0;
    descRxRead[7].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[7].xfer.decLoopCnt = 0;
    descRxRead[7].xfer.ignoreSrec = 0;
    descRxRead[7].xfer.size = ldmaCtrlSizeByte;
    descRxRead[7].xfer.structReq = 0;

    descRxRead[7].xfer.srcInc = ldmaCtrlSrcIncNone;
    descRxRead[7].xfer.dstInc = ldmaCtrlDstIncOne;
    descRxRead[7].xfer.xferCnt = 0;
    descRxRead[7].xfer.linkMode = 0;
    descRxRead[7].xfer.link = 0;
    descRxRead[7].xfer.linkAddr = 0;

    descRxRead[8].xfer.structType = ldmaCtrlStructTypeXfer;
    descRxRead[8].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;
    descRxRead[8].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs;
    descRxRead[8].xfer.structReq = 0;
    descRxRead[8].xfer.byteSwap = 0;
    descRxRead[8].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descRxRead[8].xfer.doneIfs = 0;
    descRxRead[8].xfer.reqMode = ldmaCtrlReqModeBlock;
    descRxRead[8].xfer.decLoopCnt = 0;
    descRxRead[8].xfer.ignoreSrec = 0;
    descRxRead[8].xfer.size = ldmaCtrlSizeByte;

    descRxRead[8].xfer.srcInc = ldmaCtrlSrcIncNone;
    descRxRead[8].xfer.dstInc = ldmaCtrlDstIncOne;
    descRxRead[8].xfer.xferCnt = 0;
    descRxRead[8].xfer.linkMode = 0;
    descRxRead[8].xfer.link = 0;
    descRxRead[8].xfer.linkAddr = 0;

    descTxWrite[0].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxWrite[0].xfer.size = ldmaCtrlSizeByte,
    descTxWrite[0].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxWrite[0].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxWrite[0].xfer.structReq = 0,
    descTxWrite[0].xfer.byteSwap = 0,
    descTxWrite[0].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxWrite[0].xfer.doneIfs = 0,
    descTxWrite[0].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxWrite[0].xfer.decLoopCnt = 0,
    descTxWrite[0].xfer.ignoreSrec = 0,

    descTxWrite[0].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxWrite[0].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxWrite[0].xfer.xferCnt = 0;
    descTxWrite[0].xfer.linkMode = ldmaLinkModeRel;
    descTxWrite[0].xfer.link = 1;
    descTxWrite[0].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxWrite[1].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxWrite[1].xfer.size = ldmaCtrlSizeByte,
    descTxWrite[1].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxWrite[1].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxWrite[1].xfer.structReq = 0,
    descTxWrite[1].xfer.byteSwap = 0,
    descTxWrite[1].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxWrite[1].xfer.doneIfs = 0,
    descTxWrite[1].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxWrite[1].xfer.decLoopCnt = 0,
    descTxWrite[1].xfer.ignoreSrec = 0,

    descTxWrite[1].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxWrite[1].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxWrite[1].xfer.xferCnt = 0;
    descTxWrite[1].xfer.linkMode = ldmaLinkModeRel;
    descTxWrite[1].xfer.link = 1;
    descTxWrite[1].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxWrite[2].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxWrite[2].xfer.size = ldmaCtrlSizeByte,
    descTxWrite[2].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxWrite[2].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxWrite[2].xfer.structReq = 0,
    descTxWrite[2].xfer.byteSwap = 0,
    descTxWrite[2].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxWrite[2].xfer.doneIfs = 0,
    descTxWrite[2].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxWrite[2].xfer.decLoopCnt = 0,
    descTxWrite[2].xfer.ignoreSrec = 0,

    descTxWrite[2].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxWrite[2].xfer.dstInc = ldmaCtrlDstIncOne;
    descTxWrite[2].xfer.xferCnt = 0;
    descTxWrite[2].xfer.linkMode = ldmaLinkModeRel;
    descTxWrite[2].xfer.link = 1;
    descTxWrite[2].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxWrite[3].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxWrite[3].xfer.size = ldmaCtrlSizeByte,
    descTxWrite[3].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxWrite[3].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxWrite[3].xfer.structReq = 0,
    descTxWrite[3].xfer.byteSwap = 0,
    descTxWrite[3].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxWrite[3].xfer.doneIfs = 0,
    descTxWrite[3].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxWrite[3].xfer.decLoopCnt = 0,
    descTxWrite[3].xfer.ignoreSrec = 0,

    descTxWrite[3].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxWrite[3].xfer.dstInc = ldmaCtrlDstIncNone;
    descTxWrite[3].xfer.linkMode = ldmaLinkModeRel;
    descTxWrite[3].xfer.link = 1;
    descTxWrite[3].xfer.linkAddr = 1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

    descTxWrite[4].xfer.structType = ldmaCtrlStructTypeXfer;
    descTxWrite[4].xfer.size = ldmaCtrlSizeByte,
    descTxWrite[4].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs,
    descTxWrite[4].xfer.dstAddrMode = ldmaCtrlDstAddrModeAbs,
    descTxWrite[4].xfer.structReq = 0,
    descTxWrite[4].xfer.byteSwap = 0,
    descTxWrite[4].xfer.blockSize = ldmaCtrlBlockSizeUnit1;
    descTxWrite[4].xfer.doneIfs = 0,
    descTxWrite[4].xfer.reqMode = ldmaCtrlReqModeBlock,
    descTxWrite[4].xfer.decLoopCnt = 0,
    descTxWrite[4].xfer.ignoreSrec = 0,

    descTxWrite[4].xfer.srcInc = ldmaCtrlSrcIncOne,
    descTxWrite[4].xfer.dstInc = ldmaCtrlDstIncOne;
    descTxWrite[4].xfer.xferCnt = 0;
    descTxWrite[4].xfer.linkMode = 0;
    descTxWrite[4].xfer.link = 0;
    descTxWrite[4].xfer.linkAddr = 0;

    return I2CISENABLED;

}
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

I2C_NB_RETURN_T I2C_DMA_Read(I2C_NB_READ_T *i2cNbReadInit)
{

    uint32_t i2cIrq;
    uint8_t i2cBusy = (i2cNbReadInit->i2c->STATE & 0xFF) >> 5;
    if (i2cBusy != 0)
            {
        return I2CBUSISBUSY;
    }
    else
    {
        I2C_Reset(i2cNbReadInit->i2c);
        if (i2cNbReadInit->rxCount > 1)
                {
            i2cNbReadInit->i2c->CTRL |= I2C_CTRL_AUTOACK;
        }
        i2cNbReadInit->i2c->CMD |= I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;

        int8_t count = (int8_t) i2cNbReadInit->rxCount;

        if (false) {
#if defined(I2C0)
        }
        else if (i2cNbReadInit->i2c == I2C0) {
            i2cReqTx = ldmaPeripheralSignal_I2C0_TXBL;
            i2cReqRx = ldmaPeripheralSignal_I2C0_RXDATAV;
            i2cIrq = I2C0_IRQn;
            i2cChoice = I2C0;
#endif
#if defined(I2C1)
        }
        else if (i2cNbReadInit->i2c == I2C1) {
            i2cReqTx = ldmaPeripheralSignal_I2C1_TXBL;
            i2cReqRx = ldmaPeripheralSignal_I2C1_RXDATAV;
            i2cIrq = I2C1_IRQn;
            i2cChoice = I2C1;
#endif
#if defined(I2C2)
  } else if (i2c == I2C2) {
      i2cReqTx=ldmaPeripheralSignal_I2C2_TXBL;
      i2cReqRx=ldmaPeripheralSignal_I2C2_RXDATAV;
      i2cIrq = I2C2_IRQn;
      i2cChoice = I2C2;
#endif
        }
        else {
            /* I2C clock is not defined I2C_CMD_NACK | */
            EFM_ASSERT(false);
            return I2CWRONGPORT;
        }

        uint8_t ldmaChRx = LDMA_GetFreeChannel();
        chmaskRx = 1UL << ldmaChRx;

        if (ldmaChRx == 10)
                {
            return LDMANOFREECHANNEL;
        }

        tcounter = 0;
        sladdWr = i2cNbReadInit->slaveAddress << 1;
        sladdRd = sladdWr | 1;
        last = count - 1;
        prelast = count - 2;
        txcount = i2cNbReadInit->txCount;
        tcounter = txcount + 1;
        rxcount = i2cNbReadInit->rxCount;

        int8_t jumpTxRead;
        if (i2cNbReadInit->txCount == 0)
                {
            jumpTxRead = 7;
        }
        else
        {
            jumpTxRead = 1;
        }

        descTxRead[0].xfer.srcAddr = (uint32_t) &i2cStart;
        descTxRead[0].xfer.dstAddr = (uint32_t) &i2cChoice->CMD;
        descTxRead[0].xfer.linkAddr = jumpTxRead * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

        descTxRead[1].xfer.srcAddr = (uint32_t) &sladdWr;
        descTxRead[1].xfer.dstAddr = (uint32_t) &i2cChoice->TXDATA;

        descTxRead[2].xfer.srcAddr = (uint32_t) &i2cStart;
        descTxRead[2].xfer.dstAddr = (uint32_t) &whichTrans;

        descTxRead[3].xfer.xferCnt = txcount - 1;
        descTxRead[3].xfer.srcAddr = (uint32_t) i2cNbReadInit->txBuff;
        descTxRead[3].xfer.dstAddr = (uint32_t) &i2cChoice->TXDATA;

        descTxRead[4].xfer.srcAddr = (uint32_t) &sladdRd;
        descTxRead[4].xfer.dstAddr = (uint32_t) &i2cChoice->TXDATA;

        descTxRead[5].xfer.srcAddr = (uint32_t) &tcounter;
        descTxRead[5].xfer.dstAddr = (uint32_t) &whichTrans;

        descTxRead[6].xfer.srcAddr = (uint32_t) &i2cStart;
        descTxRead[6].xfer.dstAddr = (uint32_t) &i2cChoice->CMD;

        descTxRead[7].xfer.srcAddr = (uint32_t) &sladdRd;
        descTxRead[7].xfer.dstAddr = (uint32_t) &i2cChoice->TXDATA;

        int8_t jumpRx1 = 0;
        int8_t jumpRx2 = 0;
        int8_t jumpRx3 = 0;

        switch (i2cNbReadInit->rxCount)
        {
        case 1:
            jumpRx1 = 5;
            jumpRx3 = 1;
            break;
        case 2:
            jumpRx1 = 2;
            jumpRx2 = 2;
            jumpRx3 = 2;
            break;
        default:
            jumpRx1 = 1;
            jumpRx2 = 1;
            jumpRx3 = 3;
            break;
        }

        descRxRead[0].xfer.srcAddr = (uint32_t) &txClear;
        descRxRead[0].xfer.dstAddr = (uint32_t) &i2cChoice->CMD;
        descRxRead[0].xfer.linkAddr = jumpRx1 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

        descRxRead[1].xfer.xferCnt = prelast - 1;
        descRxRead[1].xfer.srcAddr = (uint32_t) &i2cChoice->RXDATA;
        descRxRead[1].xfer.dstAddr = (uint32_t) i2cNbReadInit->rxBuff;

        descRxRead[2].xfer.srcAddr = (uint32_t) &autoA;
        descRxRead[2].xfer.dstAddr = (uint32_t) &i2cChoice->CTRL_CLR;
        descRxRead[2].xfer.linkAddr = jumpRx2 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

        descRxRead[3].xfer.srcAddr = (uint32_t) &i2cChoice->RXDATA;
        descRxRead[3].xfer.dstAddr = (uint32_t) &i2cNbReadInit->rxBuff[prelast];

        descRxRead[4].xfer.srcAddr = (uint32_t) &i2cChoice->RXDATA;
        descRxRead[4].xfer.dstAddr = (uint32_t) &i2cNbReadInit->rxBuff[0];

        descRxRead[5].xfer.srcAddr = (uint32_t) &i2cNack;
        descRxRead[5].xfer.dstAddr = (uint32_t) &i2cChoice->CMD;
        descRxRead[5].xfer.linkAddr = jumpRx3 * LDMA_DESCRIPTOR_NON_EXTEND_SIZE_WORD;

        descRxRead[6].xfer.srcAddr = (uint32_t) &i2cChoice->RXDATA;
        descRxRead[6].xfer.dstAddr = (uint32_t) &i2cNbReadInit->rxBuff[0];

        descRxRead[7].xfer.srcAddr = (uint32_t) &i2cChoice->RXDATA;
        descRxRead[7].xfer.dstAddr = (uint32_t) &i2cNbReadInit->rxBuff[1];

        descRxRead[8].xfer.srcAddr = (uint32_t) &i2cChoice->RXDATA;
        descRxRead[8].xfer.dstAddr = (uint32_t) &i2cNbReadInit->rxBuff[last];

        LDMAXBAR->CH[ldmaChRx].REQSEL = i2cReqRx;
        LDMA->CH[ldmaChRx].LINK = (uint32_t) descRxRead & _LDMA_CH_LINK_LINKADDR_MASK;
        LDMA->LINKLOAD |= chmaskRx;

        uint8_t ldmaChTx = LDMA_GetFreeChannel();
        if (ldmaChTx == 10)
                {
            return LDMAONLY1CHANNEL;
        }

        chmaskTx = 1UL << ldmaChTx;

        LDMAXBAR->CH[ldmaChTx].REQSEL = i2cReqTx;
        LDMA->CH[ldmaChTx].LINK = (uint32_t) descTxRead & _LDMA_CH_LINK_LINKADDR_MASK;
        LDMA->LINKLOAD |= chmaskTx;

        if (i2cNbReadInit->callBack != 0)
                {
            callFunc = i2cNbReadInit->callBack;
        }
        else
        {
            callFunc = 0;
        }

        I2C_Enable(i2cNbReadInit->i2c, true);

        i2cNbReadInit->i2c->IEN_CLR = _I2C_IF_RESETVALUE;
        NVIC_ClearPendingIRQ(i2cIrq);

        if (i2cNbReadInit->callBack == 0)
                {
            uint32_t i2cIf = i2cChoice->IF;
            while ((i2cIf & i2cInterrupts) == 0)
            {
                __SEV();
                __WFE();
                i2cIf = i2cChoice->IF;
            }
            if ((i2cIf & I2C_IF_MSTOP) != 0)
                    {
                LDMA->CHDIS = chmaskTx | chmaskRx;
                LDMA->CHDONE_CLR = chmaskTx | chmaskRx;
                i2cNbReadInit->i2c->CTRL_CLR = I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSE;
                return I2CTRANSFEREND;
            }
            else
            {
                LDMA->CHDIS = chmaskTx | chmaskRx;
                i2cNbReadInit->i2c->CMD |= I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
                LDMA->CHDONE_CLR = chmaskTx | chmaskRx;
                i2cNbReadInit->i2c->CTRL_CLR = I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSE;
                i2cIf = i2cIf & I2C_ERR_MASK
                ;
                switch (i2cIf)
                {
                case 128:
                    if (whichTrans == 1)
                            {
                        i2cChoice->CMD = I2C_CMD_STOP;
                        return WRONGSLAVEADDRESS;
                    }
                    else if (whichTrans > 1)
                            {
                        i2cChoice->CMD = I2C_CMD_STOP;
                        return WRONGREGISTERADDRESS;
                    }
                    else {
                        return UNKNOWNSTATE;
                    }
                case 512:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CARBITRATIONLOST;
                case 1024:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CBUSERROR;
                case 16384:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CIDLEBUSTIMEOUT;
                case 32768:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CCLOCKLOWTIMEOUT;
                case 262144:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CCLOCKLOWSTARTSTOP;

                case 524288:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CSCLERROR;

                case 1048576:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CSCLERROR;
                default:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return UNKNOWNSTATE;
                }

            }
        }
        else
        {
            I2C_IntEnable(i2cNbReadInit->i2c, i2cInterrupts);
            NVIC_ClearPendingIRQ(i2cIrq);
            NVIC_EnableIRQ(i2cIrq);
        }

        return I2CTRANSFERISSTARTED;
    }
}

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

I2C_NB_RETURN_T I2C_DMA_Write(I2C_NB_WRITE_T *i2cNbWriteInit)
{
    uint8_t i2cBusy = i2cNbWriteInit->i2c->STATE >> 5;
    if (i2cBusy != 0)
            {
        return I2CBUSISBUSY;
    }
    else
    {
        I2C_Enable(i2cNbWriteInit->i2c, false);
        i2cNbWriteInit->i2c->CTRL |= I2C_CTRL_AUTOSE;

        i2cNbWriteInit->i2c->CMD |= I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
        uint32_t i2cIrq;
        if (false) {
#if defined(I2C0)
        }
        else if (i2cNbWriteInit->i2c == I2C0) {
            i2cReqTx = ldmaPeripheralSignal_I2C0_TXBL;
            i2cIrq = I2C0_IRQn;
            i2cChoice = I2C0;
#endif
#if defined(I2C1)
        }
        else if (i2cNbWriteInit->i2c == I2C1) {
            i2cReqTx = ldmaPeripheralSignal_I2C1_TXBL;
            i2cIrq = I2C1_IRQn;
            i2cChoice = I2C1;
#endif
#if defined(I2C2)
  } else if (i2cNbWriteInit->i2c == I2C2) {
      i2cReqTx=ldmaPeripheralSignal_I2C2_TXBL;
      i2cIrq = I2C2_IRQn;
      i2cChoice = I2C2;
#endif
        }
        else {
            /* I2C clock is not defined */
            EFM_ASSERT(false);
            return I2CWRONGPORT;
        }

        sladdWr = i2cNbWriteInit->slaveAddress << 1;

        uint8_t ldmaChTx = LDMA_GetFreeChannel();
        if (ldmaChTx == 10)
                {
            return LDMANOFREECHANNEL;
        }
        chmaskTx = 1UL << ldmaChTx;

        tcounter = 0;
        txcount = i2cNbWriteInit->txCount;
        tcounter = txcount + 1;

        descTxWrite[0].xfer.srcAddr = (uint32_t) &i2cStart;
        descTxWrite[0].xfer.dstAddr = (uint32_t) &i2cChoice->CMD;

        descTxWrite[1].xfer.srcAddr = (uint32_t) &sladdWr;
        descTxWrite[1].xfer.dstAddr = (uint32_t) &i2cChoice->TXDATA;

        descTxWrite[2].xfer.srcAddr = (uint32_t) &i2cStart;
        descTxWrite[2].xfer.dstAddr = (uint32_t) &whichTrans;

        descTxWrite[3].xfer.xferCnt = txcount - 1;
        descTxWrite[3].xfer.srcAddr = (uint32_t) i2cNbWriteInit->txBuff;
        descTxWrite[3].xfer.dstAddr = (uint32_t) &i2cChoice->TXDATA;

        descTxWrite[4].xfer.srcAddr = (uint32_t) &tcounter;
        descTxWrite[4].xfer.dstAddr = (uint32_t) &whichTrans;

        LDMAXBAR->CH[ldmaChTx].REQSEL = i2cReqTx;
        LDMA->CH[ldmaChTx].LINK = (uint32_t) descTxRead & _LDMA_CH_LINK_LINKADDR_MASK;
        LDMA->LINKLOAD = chmaskTx;

        if (i2cNbWriteInit->callBack != 0)
                {
            callFunc = i2cNbWriteInit->callBack;
        }
        else
        {
            callFunc = 0;
        }
        I2C_Enable(i2cNbWriteInit->i2c, true);

        i2cNbWriteInit->i2c->IEN_CLR = _I2C_IF_RESETVALUE;
        NVIC_ClearPendingIRQ(i2cIrq);

        if (i2cNbWriteInit->callBack == 0)
                {
            uint32_t i2cIf = i2cChoice->IF;
            while ((i2cIf & i2cInterrupts) == 0)
            {
                __SEV();
                __WFE();
                i2cIf = i2cChoice->IF;
            }
            if ((i2cIf & I2C_IF_MSTOP) != 0)
                    {
                LDMA->CHDIS = chmaskTx | chmaskRx;
                LDMA->CHDONE_CLR = chmaskTx | chmaskRx;
                i2cNbWriteInit->i2c->CTRL_CLR = I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSE;
                return I2CTRANSFEREND;
            }
            else
            {
                LDMA->CHDIS = chmaskTx | chmaskRx;
                i2cNbWriteInit->i2c->CMD |= I2C_CMD_CLEARPC | I2C_CMD_CLEARTX;
                LDMA->CHDONE_CLR = chmaskTx | chmaskRx;
                i2cNbWriteInit->i2c->CTRL_CLR = I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSE;
                i2cIf = i2cIf & I2C_ERR_MASK
                ;
                switch (i2cIf)
                {
                case 128:
                    if (whichTrans == 1)
                            {
                        i2cChoice->CMD = I2C_CMD_STOP;
                        return WRONGSLAVEADDRESS;
                    }
                    else if (whichTrans > 1)
                            {
                        i2cChoice->CMD = I2C_CMD_STOP;
                        return WRONGREGISTERADDRESS;
                    }
                    else {
                        return UNKNOWNSTATE;
                    }
                case 512:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CARBITRATIONLOST;
                case 1024:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CBUSERROR;
                case 16384:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CIDLEBUSTIMEOUT;
                case 32768:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CCLOCKLOWTIMEOUT;
                case 262144:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CCLOCKLOWSTARTSTOP;

                case 524288:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CSCLERROR;

                case 1048576:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return I2CSCLERROR;
                default:
                    i2cChoice->CMD = I2C_CMD_STOP;
                    i2cChoice->CMD = I2C_CMD_ABORT;
                    return UNKNOWNSTATE;
                }

            }
        }
        else
        {
            I2C_IntEnable(i2cNbWriteInit->i2c, i2cInterrupts);
            NVIC_ClearPendingIRQ(i2cIrq);
            NVIC_EnableIRQ(i2cIrq);
        }

        return I2CTRANSFERISSTARTED;
    }
}

/**********************************************************************
 * @brief
 *  Call-back function for I2C_NB_Read and I2C_NB_Write functions.
 *
 * @param[in] callFunc
 *  Function to call after i2c transfer.
 *
 * @param[in] i2cErrStat
 *  i2c errors status.
 *
 * @return
 *  Function state.
 **********************************************************************/

static void I2CNB_callBack(uint32_t i2cErrStat, I2C_CALLFUNCTION_T callFunc)
{
    i2cErrStat = i2cErrStat & I2C_ERR_MASK;

    I2C_NB_RETURN_T err;

    switch (i2cErrStat)
    {
    case 0:
        err = CALLBACLISCALLED;
        callFunc(err);
        break;
    case 128:
        if (whichTrans == 1)
                {
            i2cChoice->CMD = I2C_CMD_STOP;
            err = WRONGSLAVEADDRESS;
            callFunc(err);
        }
        else if (whichTrans > 1)
                {
            i2cChoice->CMD = I2C_CMD_STOP;
            err = WRONGREGISTERADDRESS;
            callFunc(err);
        }
        break;
    case 512:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CARBITRATIONLOST;
        callFunc(err);
        break;
    case 1024:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CBUSERROR;
        callFunc(err);
        break;
    case 16384:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CIDLEBUSTIMEOUT;
        callFunc(err);
        break;
    case 32768:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CCLOCKLOWTIMEOUT;
        callFunc(err);
        break;
    case 262144:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CCLOCKLOWSTARTSTOP;
        callFunc(err);
        break;
    case 524288:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CSCLERROR;
        callFunc(err);
        break;
    case 1048576:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        err = I2CSCLERROR;
        callFunc(err);
        break;
    default:
        i2cChoice->CMD = I2C_CMD_STOP;
        i2cChoice->CMD = I2C_CMD_ABORT;
        break;
    }
}
