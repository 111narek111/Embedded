/*******************************************************************************
 Lynxal Inc.

 CONFIDENTIAL AND PROPRIETARY
 FOR USE BY AUTHORIZED PERSONS ONLY

 This is an unpublished work fully protected by the copyright
 laws and is a trade secret belonging to the copyright holder.

 Copyright (c) 2021-2023 Lynxal Inc. All Rights Reserved.
 ********************************************************************************
 sths34pf80.h  sths34pf80 PIR motion sensor driver definitions
 *******************************************************************************/
#ifndef STHS34PF80_H_
#define STHS34PF80_H_

#include "stdbool.h"

typedef enum {
    STHS34PF80_ODR_Freq_OFF = 0,
    STHS34PF80_ODR_Freq_0p25_Hz = 1,
    STHS34PF80_ODR_Freq_0p5_Hz,
    STHS34PF80_ODR_Freq_1_Hz,
    STHS34PF80_ODR_Freq_2_Hz,
    STHS34PF80_ODR_Freq_4_Hz,
    STHS34PF80_ODR_Freq_8_Hz,
    STHS34PF80_ODR_Freq_15_Hz,
    STHS34PF80_ODR_Freq_30_Hz
} STHS34PF80_ODR_Freq_E;

typedef enum {
    STHS34PF80_TMOS_Average_2 = 0,
    STHS34PF80_TMOS_Average_8,
    STHS34PF80_TMOS_Average_32,
    STHS34PF80_TMOS_Average_128,
    STHS34PF80_TMOS_Average_256,
    STHS34PF80_TMOS_Average_512,
    STHS34PF80_TMOS_Average_1024,
    STHS34PF80_TMOS_Average_2048,
} STHS34PF80_TMOS_Average_Rate_E;

typedef enum {
    STHS34PF80_T_Average_8 = 0,
    STHS34PF80_T_Average_4,
    STHS34PF80_T_Average_2,
    STHS34PF80_T_Average_1,
} STHS34PF80_T_Average_Rate_E;

typedef enum {
    STHS34PF80_ODR_Div_9 = 0,
    STHS34PF80_ODR_Div_20,
    STHS34PF80_ODR_Div_50,
    STHS34PF80_ODR_Div_100,
    STHS34PF80_ODR_Div_200,
    STHS34PF80_ODR_Div_400,
    STHS34PF80_ODR_Div_800,
} STHS34PF80_LPF_E;

typedef enum {
    STHS34PF80_noChip = -4,
    STHS34PF80_i2cError,
    STHS34PF80_busy,
    STHS34PF80_fail,
    STHS34PF80_done
} STHS34PF80_Return_E;

typedef enum {
    STHS34PF80_Status_Error = -1,
    STHS34PF80_Status_TambShock,
    STHS34PF80_Status_Motion,
    STHS34PF80_Status_TambShock_Motion,
    STHS34PF80_Status_Presence,
    STHS34PF80_Status_TambShock_Presence,
    STHS34PF80_Status_Motion_Presence,
    STHS34PF80_Status_All,
    STHS34PF80_Status_None
} STHS34PF80_Status_E;

typedef enum {
    STHS34PF80_Int_Mask_Error = -1,
    STHS34PF80_Int_Mask_TambShock,
    STHS34PF80_Int_Mask_Motion,
    STHS34PF80_Int_Mask_TambShock_Motion,
    STHS34PF80_Int_Mask_Presence,
    STHS34PF80_Int_Mask_TambShock_Presence,
    STHS34PF80_Int_Mask_Motion_Presence,
    STHS34PF80_Int_Mask_All,
    STHS34PF80_Int_Mask_None
} STHS34PF80_Int_Mask_E;

typedef enum {
    STHS34PF80_Int_Enable_HighImp = 0,
    STHS34PF80_Int_Enable_DataRdy,
    STHS34PF80_Int_Enable_IntMsk,
} STHS34PF80_Int_Enable_E;

typedef struct {
    bool PolarityLow;
    bool OpenDrain; //else PushPull
    STHS34PF80_Int_Mask_E IntMask;
    bool Latched;
    STHS34PF80_Int_Enable_E IntCause;
} STHS34PF80_IntConfig_S;

typedef struct {
    STHS34PF80_ODR_Freq_E ODR_Frequency;
    STHS34PF80_TMOS_Average_Rate_E TMOS_Average;
    STHS34PF80_T_Average_Rate_E T_Average;
    STHS34PF80_LPF_E LPF_High;
    STHS34PF80_LPF_E LPF_Low;
    bool BlockDataUpdate;
} STHS34PF80_Config_S;

typedef struct {
    bool Int_Pulsed;
    bool Comp_Algo;
    bool Presence_AbsTrig;
} STHS34PF80_Algorithms_S;

typedef struct {
    STHS34PF80_Config_S *Configs;
    STHS34PF80_IntConfig_S *IntConfigs;
    STHS34PF80_Algorithms_S *Algorithms;
} STHS34PF80_Init_S;

/*******************************************************************************
 * @brief
 * Reboot.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_reboot(void);

/*******************************************************************************
 * @brief
 *  Trigger One shot conversion.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_trigOneShot(void);

/*******************************************************************************
 * @brief
 *  Get status flags from sensor.
 *
 * @param[in] flags
 *  Pointer to variable where will be flags.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getStatusFlags(STHS34PF80_Status_E *flags);

/*******************************************************************************
 * @brief
 *  Get ambient temperature shock data from sensor.
 *
 * @param[in] tAmbShock
 *  Pointer to variable where will be ambient temperature shock data.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getTambShockData(int16_t *tAmbShock);

/*******************************************************************************
 * @brief
 *  Get object temperature compensation data from sensor.
 *
 * @param[in] tObjComp
 *  Pointer to variable where will be object temperature compensation data.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getTobjCompData(int16_t *tObjComp);

/*******************************************************************************
 * @brief
 *  Get object temperature data from sensor.
 *
 * @param[in] tObj
 *  Pointer to variable where will be object temperature data.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getTobjData(int16_t *tObj);

/*******************************************************************************
 * @brief
 *  Get ambient temperature data from sensor.
 *
 * @param[in] tAmb
 *  Pointer to variable where will be ambient temperature data.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getTambData(int16_t *tAmb);

/*******************************************************************************
 * @brief
 *  Get motion data from sensor.
 *
 * @param[in] motion
 *  Pointer to variable where will be motion data.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getMotionData(int16_t *motion);

/*******************************************************************************
 * @brief
 *  Get presence data from sensor.
 *
 * @param[in] presence
 *  Pointer to variable where will be presence data.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_getPresenceData(int16_t *presence);

/*******************************************************************************
 * @brief
 *  Set Thresholds for Motion and Presence.
 *
 * @param[in] thresholdPresence
 *  threshold value for presence detection.
 *
 * @param[in] thresholdMotion
 *  threshold value for motion detection.
 *
 * @return
 *  Function State
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_setThreshold(uint16_t thresholdPresence, uint16_t thresholdMotion, uint16_t thresholdTambShock);

/*******************************************************************************
 * @brief
 *  Initialize STHS34PF80 Motion sensor.
 *
 * @param[in] init
 *  Structure for sensor initialization.
 *
 * @return
 *  Function State
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_init(STHS34PF80_Init_S *init);

#endif /* STHS34PF80_H_ */
