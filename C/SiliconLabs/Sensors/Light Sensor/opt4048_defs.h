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


#ifndef __OPT4048_DEFS_H__
#define __OPT4048_DEFS_H__

#define OPT4048_CHANNEL0_REG1   0x00
#define OPT4048_CHANNEL0_REG2   0x01
#define OPT4048_CHANNEL1_REG1   0x02
#define OPT4048_CHANNEL1_REG2   0x03
#define OPT4048_CHANNEL2_REG1   0x04
#define OPT4048_CHANNEL2_REG2   0x05
#define OPT4048_CHANNEL3_REG1   0x06
#define OPT4048_CHANNEL3_REG2   0x07
#define OPT4048_THRESHOLD_L_REG 0x08
#define OPT4048_THRESHOLD_H_REG 0x09
#define OPT4048_CONFIG_REG      0x0A
#define OPT4048_INTCONFIG_REG   0x0B
#define OPT4048_FLAG_REG        0x0C
#define OPT4048_DEVICE_ID_REG   0x11

/* All Data Registers */
#define OPT4048_MSB_MASK        0x0FFF

/* OPT4048_CONFIG_REG: RANGE */
#define OPT4048_CONFIG_QWAKEUP              1
#define OPT4048_CONFIG_NO_QWAKEUP           0
/* OPT4048_CONFIG_REG: Reserved */
#define OPT4048_CONFIG_RESERVE_0            0
/* OPT4048_CONFIG_REG: RANGE */
#define OPT4048_RANGE_2_2_kLux              0
#define OPT4048_RANGE_4_5_kLux              1
#define OPT4048_RANGE_9_kLux                2
#define OPT4048_RANGE_18_kLux               3
#define OPT4048_RANGE_36_kLux               4
#define OPT4048_RANGE_72_kLux               5
#define OPT4048_RANGE_144_kLux              6
#define OPT4048_RANGE_AUTOSCALE             12
/* OPT4048_CONFIG_REG: CONVERSION_TIME */
#define OPT4048_CONFIG_CONV_TIME_600uS      0   /* 281.8 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_1mS        1   /* 140.9 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_1_8mS      2   /* 70.5 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_3_4mS      3   /* 35.2 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_6_5_mS     4   /* 17.6 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_12_7_mS    5   /* 8.8 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_25_mS      6   /* 4.4 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_50_mS      7   /* 2.2 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_100_mS     8   /* 1.1 Lux LSB */
#define OPT4048_CONFIG_CONV_TIME_200_mS     9   /* 550.4 mLux LSB */
#define OPT4048_CONFIG_CONV_TIME_400_mS     10  /* 275.2 mLux LSB */
#define OPT4048_CONFIG_CONV_TIME_800_mS     11  /* 137.6 mLux LSB */
/* OPT4048_CONFIG_REG: OPERATING_MODE bits */
#define OPT4048_CONFIG_MODE_POWER_DOWN      0
#define OPT4048_CONFIG_MODE_FORCE_AUTO      1
#define OPT4048_CONFIG_MODE_ONE_SHOT        2
#define OPT4048_CONFIG_MODE_CONTINUOUS      3
/* OPT4048_CONFIG_REG: LATCH */
#define OPT4048_CONFIG_LATCH_HYSTERESYS     0
#define OPT4048_CONFIG_LATCH_WINDOW         1
/* OPT4048_CONFIG_REG: INT_POL */
#define OPT4048_CONFIG_INT_POL_ACTIVE_LOW   0
#define OPT4048_CONFIG_INT_POL_ACTIVE_HIGH  1
/* OPT4048_CONFIG_REG: FAULT_COUNT */
#define OPT4048_CONFIG_FAULT_COUNT_ONE      0
#define OPT4048_CONFIG_FAULT_COUNT_TWO      1
#define OPT4048_CONFIG_FAULT_COUNT_FOUR     2
#define OPT4048_CONFIG_FAULT_COUNT_EIGHT    3

/* OPT4048_INTCONGIF_REG: Reserved */
#define OPT4048_INT_CFG_DEFAULT0            0x100
/* OPT4048_INTCONGIF_REG: THRESHOLD_CH_SEL */
#define OPT4048_INT_THRESHOLD_CH_0          0
#define OPT4048_INT_THRESHOLD_CH_1          1
#define OPT4048_INT_THRESHOLD_CH_2          2
#define OPT4048_INT_THRESHOLD_CH_3          3
/* OPT4048_INTCONGIF_REG: INT_DIR */
#define OPT4048_INT_DIR_INPUT               0
#define OPT4048_INT_DIR_OUTPUT              1
/* OPT4048_INTCONGIF_REG: INT_CFG */
#define OPT4048_INT_CFG_SMBUS_ALERT         0
#define OPT4048_INT_CFG_INT_ON_CHANNEL      1
#define OPT4048_INT_CFG_INT_ON_ALL_CHANNELS 3
/* OPT4048_INTCONGIF_REG: Reserved */
#define OPT4048_INT_CFG_RESERVED            0
/* OPT4048_INTCONGIF_REG: I2C_BURST */
#define OPT4048_I2C_BURST_SINGLE_ACCESS     0
#define OPT4048_I2C_BURST_AUTO_INCREMENT    1

/* OPT4048_DEVICE_ID_REG */
#define OPT4048_DEVICE_ID                   0x0821


#define OPT4048_ATTENUATION_FACTOR  1.0
#define OPT4048_INT_POLARITY        OPT4048_CONFIG_INT_POL_ACTIVE_LOW
#define OPT4048_CONFIG_CONV_TIME    OPT4048_CONFIG_CONV_TIME_200_mS
#define OPT4048_I2C_ADDRESS         0x44

#endif /* __OPT4048_DEFS_H__ */
