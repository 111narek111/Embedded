/***************************************************************************//**
 * @file
 * @brief Top level application functions for OPT4048 light sensor driver
 *******************************************************************************
 * Description
 *
 * This file contains the top-level functions for configuring and controlling
 * the OPT4048 light sensor on Silicon Labs devices. It provides initialization
 * routines, functions for reading light intensity values,CIE 1931 xy coordinates,
 * Light CCT and other utility functions to manage sensor calibration and data processing.
 *
 * Author
 * Narek Hakobyan
 ******************************************************************************/
#include "opt4048.h"
#include "opt4048_defs.h"
#include "stdbool.h"

float matrixCIEL[4][4] = { { 234.892992, -189.652390, 120.811684, 0 },
    { 407.467441, 198.958202, -158.848115, 0.00215 },
    { 928.619404, -169.739553, 674.021520, 0 },
    { 0, 0, 0, 0 } };

/*******************************************************************************
 * @brief
 *  Get Lux data.
 *
 * @param[in] lux
 *  Pointer to variable where will be lux data.
 *
 * @return
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_GetLux(uint32_t *lux)
{
    uint32_t adc_codes_ch1 = 0;
    uint8_t reg_name = OPT4048_CHANNEL1_REG1;
    uint8_t rxBuf[4];

    I2C_DMA_Read_S i2c_read = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = rxBuf,
        .rxCount = 4
    };

    if(I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    adc_codes_ch1 = ((((rxBuf[0] & 0x0F) << 8) | rxBuf[1]) << 8) | rxBuf[2];
    adc_codes_ch1 = adc_codes_ch1 << (rxBuf[0] >> 4);

    *lux = (adc_codes_ch1 * 215) / 100000;

    return OPT4048_dataReady;

}

/*******************************************************************************
 * @brief
 *  Get CIE1931 coordinates.
 *
 * @param[in] cie
 *  Array of 2 elements for CIE1931 data.
 *
 * @return
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_GetCIE(float *cie)
{
    uint32_t adc_codes_ch0 = 0;
    uint32_t adc_codes_ch1 = 0;
    uint32_t adc_codes_ch2 = 0;
    uint32_t adc_codes_ch3 = 0;
    float x, y, z;
    uint8_t rxBuf[16];
    uint8_t reg_name = OPT4048_CHANNEL0_REG1;

    I2C_DMA_Read_S i2c_read = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = rxBuf,
        .rxCount = 16
    };

    if(I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    adc_codes_ch0 = ((((rxBuf[0] & 0x0F) << 8) | rxBuf[1]) << 8) | rxBuf[2];
    adc_codes_ch0 = adc_codes_ch0 << (rxBuf[0] >> 4);
    adc_codes_ch1 = ((((rxBuf[4] & 0x0F) << 8) | rxBuf[5]) << 8) | rxBuf[6];
    adc_codes_ch1 = adc_codes_ch1 << (rxBuf[4] >> 4);
    adc_codes_ch2 = ((((rxBuf[8] & 0x0F) << 8) | rxBuf[9]) << 8) | rxBuf[10];
    adc_codes_ch2 = adc_codes_ch2 << (rxBuf[8] >> 4);
    adc_codes_ch3 = ((((rxBuf[12] & 0x0F) << 8) | rxBuf[13]) << 8) | rxBuf[14];
    adc_codes_ch3 = adc_codes_ch3 << (rxBuf[12] >> 4);

    x = ((float) adc_codes_ch0 * matrixCIEL[0][0]) / 1000000 + ((float) adc_codes_ch1 * matrixCIEL[1][0]) / 10000000 + ((float) adc_codes_ch2 * matrixCIEL[2][0]) / 10000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][0]);
    y = ((float) adc_codes_ch0 * matrixCIEL[0][1]) / 10000000 + ((float) adc_codes_ch1 * matrixCIEL[1][1]) / 1000000 + ((float) adc_codes_ch2 * matrixCIEL[2][1]) / 10000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][1]);
    z = ((float) adc_codes_ch0 * matrixCIEL[0][2]) / 10000000 + ((float) adc_codes_ch1 * matrixCIEL[1][2]) / 10000000 + ((float) adc_codes_ch2 * matrixCIEL[2][2]) / 1000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][2]);

    cie[0] = x / (x + y + z);
    cie[1] = y / (x + y + z);

    return OPT4048_dataReady;

}

/*******************************************************************************
 * @brief
 *  Get CCT data.
 *
 * @param[in] cct
 *  Pointer to variable where will be cct data.
 *
 * @return
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_GetCCT(float *cct)
{

    uint32_t adc_codes_ch0 = 0;
    uint32_t adc_codes_ch1 = 0;
    uint32_t adc_codes_ch2 = 0;
    uint32_t adc_codes_ch3 = 0;
    float x, y, z;
    float ciex, ciey;
    float n;
    uint8_t rxBuf[16];
    uint8_t reg_name = OPT4048_CHANNEL0_REG1;

    I2C_DMA_Read_S i2c_read = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = rxBuf,
        .rxCount = 16
    };

    if(I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    adc_codes_ch0 = ((((rxBuf[0] & 0x0F) << 8) | rxBuf[1]) << 8) | rxBuf[2];
    adc_codes_ch0 = adc_codes_ch0 << (rxBuf[0] >> 4);
    adc_codes_ch1 = ((((rxBuf[4] & 0x0F) << 8) | rxBuf[5]) << 8) | rxBuf[6];
    adc_codes_ch1 = adc_codes_ch1 << (rxBuf[4] >> 4);
    adc_codes_ch2 = ((((rxBuf[8] & 0x0F) << 8) | rxBuf[9]) << 8) | rxBuf[10];
    adc_codes_ch2 = adc_codes_ch2 << (rxBuf[8] >> 4);
    adc_codes_ch3 = ((((rxBuf[12] & 0x0F) << 8) | rxBuf[13]) << 8) | rxBuf[14];
    adc_codes_ch3 = adc_codes_ch3 << (rxBuf[12] >> 4);
    x = ((float) adc_codes_ch0 * matrixCIEL[0][0]) / 1000000 + ((float) adc_codes_ch1 * matrixCIEL[1][0]) / 10000000 + ((float) adc_codes_ch2 * matrixCIEL[2][0]) / 10000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][0]);
    y = ((float) adc_codes_ch0 * matrixCIEL[0][1]) / 10000000 + ((float) adc_codes_ch1 * matrixCIEL[1][1]) / 1000000 + ((float) adc_codes_ch2 * matrixCIEL[2][1]) / 10000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][1]);
    z = ((float) adc_codes_ch0 * matrixCIEL[0][2]) / 10000000 + ((float) adc_codes_ch1 * matrixCIEL[1][2]) / 10000000 + ((float) adc_codes_ch2 * matrixCIEL[2][2]) / 1000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][2]);

    ciex = x / (x + y + z);
    ciey = y / (x + y + z);
    n = (ciex - 0.332) / (0.1858 - ciey);

    *cct = 437 * pow(n, 3) + 3601 * pow(n, 2) + 6861 * n + 5517;

    return OPT4048_dataReady;
}

/*******************************************************************************
 * @brief
 *  Get All of the data.
 *
 * @param[in] lux
 *  Pointer to variable where will be lux data.
 *
 * @param[in] cie
 *  Array of 2 elements for CIE1931 data.
 *
 * @param[in] cct
 *  Pointer to variable where will be cct data.
 *
 * @return
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_GetAllData(uint32_t *lux, float *cie, float *cct)
{
    uint32_t adc_codes_ch0 = 0;
    uint32_t adc_codes_ch1 = 0;
    uint32_t adc_codes_ch2 = 0;
    uint32_t adc_codes_ch3 = 0;
    float x, y, z;
    float n;
    uint8_t rxBuf[16];
    uint8_t reg_name = OPT4048_CHANNEL0_REG1;

    I2C_DMA_Read_S i2c_read = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = rxBuf,
        .rxCount = 16
    };

    if(I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    adc_codes_ch0 = ((((rxBuf[0] & 0x0F) << 8) | rxBuf[1]) << 8) | rxBuf[2];
    adc_codes_ch0 = adc_codes_ch0 << (rxBuf[0] >> 4);
    adc_codes_ch1 = ((((rxBuf[4] & 0x0F) << 8) | rxBuf[5]) << 8) | rxBuf[6];
    adc_codes_ch1 = adc_codes_ch1 << (rxBuf[4] >> 4);
    adc_codes_ch2 = ((((rxBuf[8] & 0x0F) << 8) | rxBuf[9]) << 8) | rxBuf[10];
    adc_codes_ch2 = adc_codes_ch2 << (rxBuf[8] >> 4);
    adc_codes_ch3 = ((((rxBuf[12] & 0x0F) << 8) | rxBuf[13]) << 8) | rxBuf[14];
    adc_codes_ch3 = adc_codes_ch3 << (rxBuf[12] >> 4);

    *lux = (adc_codes_ch1 * 215) / 100000;
    x = ((float) adc_codes_ch0 * matrixCIEL[0][0]) / 1000000 + ((float) adc_codes_ch1 * matrixCIEL[1][0]) / 10000000 + ((float) adc_codes_ch2 * matrixCIEL[2][0]) / 10000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][0]);
    y = ((float) adc_codes_ch0 * matrixCIEL[0][1]) / 10000000 + ((float) adc_codes_ch1 * matrixCIEL[1][1]) / 1000000 + ((float) adc_codes_ch2 * matrixCIEL[2][1]) / 10000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][1]);
    z = ((float) adc_codes_ch0 * matrixCIEL[0][2]) / 10000000 + ((float) adc_codes_ch1 * matrixCIEL[1][2]) / 10000000 + ((float) adc_codes_ch2 * matrixCIEL[2][2]) / 1000000
            + ((float) adc_codes_ch3 * matrixCIEL[3][2]);
    cie[0] = x / (x + y + z);
    cie[1] = y / (x + y + z);
    n = (cie[0] - 0.332) / (0.1858 - cie[1]);
    *cct = 437 * (n * n * n) + 3601 * (n * n) + 6861 * n + 5517;

    return OPT4048_dataReady;

}

/*******************************************************************************
 * @brief
 *  Get flags.
 *
 * @return
 *  Flags.
 *******************************************************************************/

OPT4048_Return_E OPT4048_IntGet(uint8_t *flag)
{
    uint8_t reg_name = OPT4048_FLAG_REG;

    I2C_DMA_Read_S i2c_read = {
        .i2c = I2C0,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = flag,
        .rxCount = 1
    };

    if(I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    return OPT4048_dataReady;
}

/*******************************************************************************
 * @brief
 *  Set thresholds.
 *
 * @param[in] threshold_l
 *  Thresholds LSB.
 *
 * @param[in] threshold_h
 *  Thresholds MSB.
 *
 * @return
 *  Function state.
 *******************************************************************************/

OPT4048_Return_E OPT4048_ThresholdSet(uint8_t *threshold_l, uint8_t *threshold_h)
{
    uint8_t txData[2] = { OPT4048_THRESHOLD_L_REG, *threshold_l };

    I2C_DMA_Write_S i2c_write = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = txData,
        .txCount = 2,
    };

    if(I2C_DMA_Write(&i2c_write) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    txData[0] = OPT4048_THRESHOLD_H_REG;
    txData[1] = *threshold_h;

    if(I2C_DMA_Write(&i2c_write) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    return OPT4048_dataReady;
}

/*******************************************************************************
 * @brief
 *  Initialize OPT4048 Light sensor.
 *
 * @param[in] init
 *  structure for sensor initialization.
 *
 * @return
 *  Function state.
 *******************************************************************************/

OPT4048_Return_E OPT4048_Init(OPT4048_Init_S *init)
{
    I2C_DMA_Init_S i2c_init = {
        .i2c = OPT4048_I2C_PORT,
        .master = true,
        .ldmaInit = LDMA_INIT_DEFAULT,
        .freq = 100000,
        .refFreq = 0,
        .SDAport = OPT4048_I2C_SDA_PORT,
        .SDApin = OPT4048_I2C_SDA_PIN,
        .SCLport = OPT4048_I2C_SCL_PORT,
        .SCLpin = OPT4048_I2C_SCL_PIN,
    };
    if(I2C_DMA_Init(&i2c_init) != I2CISENABLED){
        return OPT4048_i2cError;
    }

    uint8_t reg_name = OPT4048_DEVICE_ID_REG;
    uint8_t rxBuf[2] = {0};
    uint16_t chipId = 0;

    I2C_DMA_Read_S i2c_read = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = rxBuf,
        .rxCount = 2
    };

    if(I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    chipId = (((uint16_t) rxBuf[0] << 8) | rxBuf[1]);

    if(chipId != OPT4048_DEVICE_ID){
        return OPT4048_noChip;
    }

    uint8_t cfg[2];
    uint8_t int_cfg[2];

    cfg[0] = (init->config->qWake << 7) | (init->config->range << 2) | (init->config->convTime >> 2);
    cfg[1] = (init->config->convTime << 6) | (init->config->operatingMode << 4) | (init->config->latch << 3) | (init->config->intPol << 2) | (init->config->faultCount);
    int_cfg[0] = 128;
    int_cfg[1] = (init->intConfig->thresholdChannel << 5) | (init->intConfig->interruptDirection << 4) | (init->intConfig->interruptConfiguration << 2)
            | (init->intConfig->i2cBurst);

    uint8_t txData[3] = { OPT4048_CONGIF_REG, cfg[0], cfg[1] };

    I2C_DMA_Write_S i2c_write = {
        .i2c = OPT4048_I2C_PORT,
        .slaveAddress = OPT4048_I2C_ADDRESS,
        .txBuff = txData,
        .txCount = 3,
    };

    if(I2C_DMA_Write(&i2c_write) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    txData[0] = OPT4048_INTCONGIF_REG;
    txData[1] = int_cfg[0];
    txData[2] = int_cfg[1];

    if(I2C_DMA_Write(&i2c_write) != I2CTRANSFERISSTARTED){
        return OPT4048_i2cError;
    }

    return OPT4048_initDone;

}
