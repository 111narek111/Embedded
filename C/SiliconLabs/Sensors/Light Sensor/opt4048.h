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
#ifndef OPT4048_H_
#define OPT4048_H_

#include "i2c_dma.h"
#include <math.h>

#define SLAVE_ADDR_GND 0x44
#define SLAVE_ADDR_VDD 0x45
#define SLAVE_ADDR_SDA 0x46
#define SLAVE_ADDR_SCL 0x45

#define OPT4048_I2C_PORT        I2C0
#define OPT4048_I2C_SDA_PORT    gpioPortC
#define OPT4048_I2C_SDA_PIN     5
#define OPT4048_I2C_SCL_PORT    gpioPortC
#define OPT4048_I2C_SCL_PIN     6

#define OPT4048_THRESHOLD_L_REG 0x08
#define OPT4048_THRESHOLD_H_REG 0x09
#define OPT4048_CONGIF_REG      0x0A
#define OPT4048_INTCONGIF_REG   0x0B
#define OPT4048_FLAG_REG        0x0C

//only in standby mode
typedef enum {
    OPT4048_noQWake = 0,
    OPT4048_QWake = 1,
} OPT4048_QWake_E;

typedef enum {
    OPT4048_lux2254 = 0,
    OPT4048_lux4509 = 1,
    OPT4048_lux9018 = 2,
    OPT4048_lux18036 = 3,
    OPT4048_lux36071 = 4,
    OPT4048_lux72142 = 5,
    OPT4048_lux144284 = 6,
    OPT4048_autoScale = 7
} OPT4048_Range_E;

typedef enum {
    OPT4048_us600 = 0,
    OPT4048_ms1 = 1,
    OPT4048_ms1p8 = 2,
    OPT4048_ms3p4 = 3,
    OPT4048_ms6p5 = 4,
    OPT4048_ms12p7 = 5,
    OPT4048_ms25 = 6,
    OPT4048_ms50 = 7,
    OPT4048_ms100 = 8,
    OPT4048_ms200 = 9,
    OPT4048_ms400 = 10,
    OPT4048_ms800 = 11
} OPT4048_Conversion_Time_E;

typedef enum {
    OPT4048_powerDown = 0,
    OPT4048_forcedAutoRangeOneShot = 1,
    OPT4048_oneShot = 2,
    OPT4048_continuous = 3
} OPT4048_OperatingMode_E;

//??
typedef enum {
    OPT4048_transparentHysteresis = 0,
    OPT4048_latchedWindow = 1
} OPT4048_Latch_E;

typedef enum {
    OPT4048_activeLow = 0,
    OPT4048_activeHigh = 1,
} OPT4048_IntPol_E;

typedef enum {
    OPT4048_oneFault = 0,
    OPT4048_twoFault = 1,
    OPT4048_fourFault = 2,
    OPT4048_eightFault = 3
} OPT4048_FaultCount_E;

typedef struct {
    OPT4048_QWake_E qWake;
    OPT4048_Range_E range;
    OPT4048_Conversion_Time_E convTime;
    OPT4048_OperatingMode_E operatingMode;
    OPT4048_Latch_E latch;
    OPT4048_IntPol_E intPol;
    OPT4048_FaultCount_E faultCount;
} OPT4048_Config_S;

typedef enum {
    OPT4048_channel0 = 0,
    OPT4048_channel1 = 1,
    OPT4048_channel2 = 2,
    OPT4048_channel3 = 3
} OPT4048_ThresholdCh_E;

typedef enum {
    OPT4048_input = 0,
    OPT4048_output = 1,
} OPT4048_IntDir_E;

typedef enum {
    OPT4048_SMBUS = 0,
    OPT4048_nextChannel = 1,
    OPT4048_allChannels = 3
} OPT4048_IntCfg_E;

typedef enum {
    OPT4048_normalRead = 0,
    OPT4048_i2cBust = 1,
} OPT4048_I2CBurst_E;

typedef struct {
    OPT4048_ThresholdCh_E thresholdChannel;
    OPT4048_IntDir_E interruptDirection;
    OPT4048_IntCfg_E interruptConfiguration;
    OPT4048_I2CBurst_E i2cBurst;
} OPT4048_IntConfig_S;

typedef struct {
    OPT4048_Config_S *config;
    OPT4048_IntConfig_S *intConfig;
} OPT4048_Init_S;

typedef enum {
    OPT4048_noChip = -3,
    OPT4048_i2cError = -2,
    OPT4048_dataError = -1,
    OPT4048_dataReady = 0,
    OPT4048_initDone = 1,
} OPT4048_Return_E;


/*******************************************************************************
 * @brief
 *  Initialize OPT4048 Light sensor.
 *
 * @param[in] init
 *  Structure for sensor initialization.
 *
 * @return
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_Init(OPT4048_Init_S *init);

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
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_ThresholdSet(uint8_t *threshold_l, uint8_t *threshold_h);

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

OPT4048_Return_E OPT4048_GetLux(uint32_t *lux);

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

OPT4048_Return_E OPT4048_GetCIE(float *cie);

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

OPT4048_Return_E OPT4048_GetCCT(float *cct);

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

OPT4048_Return_E OPT4048_GetAllData(uint32_t *lux, float *cie, float *cct);

/*******************************************************************************
 * @brief
 *  Get flags.
 *
 * @param[in] flag
 *  Pointer to variable where will be flags.
 *
 * @return
 *  Function State
 *******************************************************************************/

OPT4048_Return_E OPT4048_IntGet(uint8_t *flag);

#endif /* OPT4048_H_ */
