/*******************************************************************************
 Lynxal Inc.

 CONFIDENTIAL AND PROPRIETARY
 FOR USE BY AUTHORIZED PERSONS ONLY

 This is an unpublished work fully protected by the copyright
 laws and is a trade secret belonging to the copyright holder.

 Copyright (c) 2021-2023 Lynxal Inc. All Rights Reserved.
 ********************************************************************************
 sths34pf80.c  sths34pf80 PIR motion sensor driver
 *******************************************************************************/
#include "sths34pf80_defs.h"
#include "sths34pf80.h"
#include "i2c_dma.h"

#define STHS34PF80_FLAG_TAMB_SHOCK              (1 << 0)
#define STHS34PF80_FLAG_MOTION                  (1 << 1)
#define STHS34PF80_FLAG_PRESENCE                (1 << 2)
#define STHS34PF80_FLAGS_MASK                   (\
                                                    STHS34PF80_FLAG_TAMB_SHOCK |\
                                                    STHS34PF80_FLAG_MOTION |\
                                                    STHS34PF80_FLAG_PRESENCE \
                                                )

#define STHS34PF80_CTRL2_FUNC_CFG_ACCESS        (1 << 4)
#define STHS34PF80_EMB_PAGE_FUNC_CFG_READ       (1 << 5)
#define STHS34PF80_EMB_PAGE_FUNC_CFG_WRITE      (1 << 6)

#define STHS34PF80_STARTUP_DELAY_uSEC           3000

#define STHS34PF80_SET_HYST(threshold)          (\
                                                    (\
                                                        (((double)(threshold) - 90) / (STHS34PF80_THRESHOLD_MAX - STHS34PF80_THRESHOLD_MIN)) * 110 \
                                                    ) + 40 \
                                                )

uint8_t Current_freq = 0;

/*******************************************************************************
 * @brief
 *  Power downs sensor.
 *
 * @return
 *  Function state.
 *******************************************************************************/

static STHS34PF80_Return_E sths34pf80_powerDoWn()
{
    uint8_t txBuffer[2] = { STHS34PF80_CTRL1_REG, STHS34PF80_CTRL1_SET_POWER_DOWN };

    I2C_DMA_Write_S i2c_write = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = txBuffer,
        .txCount = 2,
    };

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    return STHS34PF80_done;
}

/*******************************************************************************
 * @brief
 *  Write to sensors embedded registers.
 *
 * @return
 *  Function state.
 *******************************************************************************/

static STHS34PF80_Return_E sths34pf80_write_emb_reg(uint8_t address, uint8_t *value, uint8_t size)
{
    uint8_t reg[2] = { STHS34PF80_CTRL2_REG, STHS34PF80_CTRL2_FUNC_CFG_ACCESS };

    I2C_DMA_Write_S i2c_write = {
        .i2c = I2C0,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = reg,
        .txCount = 2,
    };

    /* Enable access to embedded functions registers. */
    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    /* Enables the write procedure for the embedded functions. */
    reg[0] = STHS34PF80_EMB_PAGE_RW_REG;
    reg[1] = STHS34PF80_EMB_PAGE_FUNC_CFG_WRITE;

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = STHS34PF80_EMB_FUNC_CFG_ADDR_REG;
    reg[1] = address;

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = STHS34PF80_EMB_FUNC_CFG_DATA_REG;

    for (int i = 0; i < size; i++) {
        reg[1] = value[i];
        if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
            return STHS34PF80_i2cError;
        }
    }
    /* Disable RW access */
    reg[0] = STHS34PF80_EMB_PAGE_RW_REG;
    reg[1] = 0;
    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = STHS34PF80_CTRL2_REG;
    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }
    return STHS34PF80_done;
}

/*******************************************************************************
 * @brief
 * Reboot.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_reboot(void)
{
    uint8_t reg[2] = {STHS34PF80_CTRL2_REG, STHS34PF80_CTRL2_REBOOT};

     I2C_DMA_Write_S i2c_write = {
         .i2c = I2C0,
         .slaveAddress = STHS34PF80_I2C_ADDRESS,
         .txBuff = reg,
         .txCount = 2,
     };

     if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
         return STHS34PF80_i2cError;
     }

    return STHS34PF80_done;
}

/*******************************************************************************
 * @brief
 *  Trigger One shot conversion.
 *
 * @return
 *  Function state.
 *******************************************************************************/

STHS34PF80_Return_E sths34pf80_trigOneShot(void)
{
    uint8_t reg[2] = {STHS34PF80_CTRL2_REG, STHS34PF80_CTRL2_TRIGGER_ONE_SHOT};

     I2C_DMA_Write_S i2c_write = {
         .i2c = I2C0,
         .slaveAddress = STHS34PF80_I2C_ADDRESS,
         .txBuff = reg,
         .txCount = 2,
     };

     if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
         return STHS34PF80_i2cError;
     }

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getStatusFlags(STHS34PF80_Status_E *flags)
{
    uint8_t reg_name = STHS34PF80_FUNC_STATUS_REG;
    uint8_t data;

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = &data,
        .rxCount = 1
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!flags) {
        return STHS34PF80_fail;
    }
    *flags = data;

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getTambShockData(int16_t *tAmbShock)
{
    uint8_t reg_name = STHS34PF80_TAMB_SHOCK_L_REG;
    uint8_t data[2];

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = data,
        .rxCount = 2
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!tAmbShock) {
        return STHS34PF80_fail;
    }
    *tAmbShock = (((int16_t) data[1] << 8) | data[0]);

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getTobjCompData(int16_t *tObjComp)
{
    uint8_t reg_name = STHS34PF80_TOBJ_COMP_L_REG;
    uint8_t data[2];

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = data,
        .rxCount = 2
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!tObjComp) {
        return STHS34PF80_fail;
    }
    *tObjComp = (((int16_t) data[1] << 8) | data[0]);

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getTobjData(int16_t *tObj)
{
    uint8_t reg_name = STHS34PF80_TOBJECT_L_REG;
    uint8_t data[2];

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = data,
        .rxCount = 2
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!tObj) {
        return STHS34PF80_fail;
    }
    *tObj = (((int16_t) data[1] << 8) | data[0]);

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getTambData(int16_t *tAmb)
{
    uint8_t reg_name = STHS34PF80_TAMBIENT_L_REG;
    uint8_t data[2];

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = data,
        .rxCount = 2
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!tAmb) {
        return STHS34PF80_fail;
    }
    *tAmb = (((int16_t) data[1] << 8) | data[0]);

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getMotionData(int16_t *motion)
{
    uint8_t reg_name = STHS34PF80_TMOTION_L_REG;
    uint8_t data[2];

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = data,
        .rxCount = 2
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!motion) {
        return STHS34PF80_fail;
    }
    *motion = (((int16_t) data[1] << 8) | data[0]);

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_getPresenceData(int16_t *presence)
{
    uint8_t reg_name = STHS34PF80_TPRESENCE_L_REG;
    uint8_t data[2];

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = data,
        .rxCount = 2
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (!presence) {
        return STHS34PF80_fail;
    }
    *presence = (((int16_t) data[1] << 8) | data[0]);

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_setThreshold(uint16_t thresholdPresence, uint16_t thresholdMotion, uint16_t thresholdTambShock)
{
    if ((thresholdPresence > STHS34PF80_THRESHOLD_MAX) || (thresholdPresence < STHS34PF80_THRESHOLD_MIN)) {
        return STHS34PF80_fail;
    }

    if ((thresholdMotion > STHS34PF80_THRESHOLD_MAX) || (thresholdMotion < STHS34PF80_THRESHOLD_MIN)) {
        return STHS34PF80_fail;
    }

    if ((thresholdTambShock > STHS34PF80_THRESHOLD_MAX) || (thresholdTambShock < STHS34PF80_THRESHOLD_MIN)) {
        return STHS34PF80_fail;
    }

    /* Enter Power down */
    sths34pf80_powerDoWn();

    uint8_t reg[2] = { STHS34PF80_FUNC_STATUS_REG, 0 };

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg[0],
        .txCount = 1,
        .rxBuff = &reg[1],
        .rxCount = 1
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    /* Set thresholds */
    {
        reg[0] = (uint8_t) (thresholdPresence & 0xFF);
        reg[1] = (uint8_t) (thresholdPresence >> 8);
        if (sths34pf80_write_emb_reg(STHS34PF80_EMB_PRESENCE_THS_REG1, reg, 2) != STHS34PF80_done) {
            return STHS34PF80_fail;
        }

        reg[0] = (uint8_t) (thresholdMotion & 0xFF);
        reg[1] = (uint8_t) (thresholdMotion >> 8);
        if (sths34pf80_write_emb_reg(STHS34PF80_EMB_MOTION_THS_REG1, reg, 2) != STHS34PF80_done) {
            return STHS34PF80_fail;
        }

        reg[0] = (uint8_t) (thresholdTambShock & 0xFF);
        reg[1] = (uint8_t) (thresholdTambShock >> 8);
        if (sths34pf80_write_emb_reg(STHS34PF80_EMB_TAMB_SHOCK_THS_REG1, reg, 2) != STHS34PF80_done) {
            return STHS34PF80_fail;
        }
    }

    /* Set hysteresis */
    {
        reg[0] = (uint8_t) STHS34PF80_SET_HYST(thresholdPresence);
        if (sths34pf80_write_emb_reg(STHS34PF80_EMB_HYST_PRESENCE_REG, reg, 1) != STHS34PF80_done) {
            return STHS34PF80_fail;
        }
        reg[0] = (uint8_t) STHS34PF80_SET_HYST(thresholdMotion);
        if (sths34pf80_write_emb_reg(STHS34PF80_EMB_HYST_MOTION_REG, reg, 1) != STHS34PF80_done) {
            return STHS34PF80_fail;
        }
        reg[0] = (uint8_t) STHS34PF80_SET_HYST(thresholdTambShock);
        if (sths34pf80_write_emb_reg(STHS34PF80_EMB_TAMB_SHOCK_THS_REG1, reg, 1) != STHS34PF80_done) {
            return STHS34PF80_fail;
        }

    }

    reg[0] = STHS34PF80_EMB_RESET_ALGO;
    if (sths34pf80_write_emb_reg(STHS34PF80_EMB_RESET_ALGO_REG, reg, 1) != STHS34PF80_done) {
        return STHS34PF80_fail;
    }

    /* Restore ODR */
    reg[0] = STHS34PF80_CTRL1_REG;
    reg[1] = Current_freq;

    I2C_DMA_Write_S i2c_write = {
        .i2c = I2C0,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = reg,
        .txCount = 2,
    };

    /* Enable access to embedded functions registers. */
    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    return STHS34PF80_done;
}

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

STHS34PF80_Return_E sths34pf80_init(STHS34PF80_Init_S *init)
{
    I2C_DMA_Init_S i2c_init = {
        .i2c = STHS34PF80_I2C_PORT,
        .master = true,
        .ldmaInit = LDMA_INIT_DEFAULT,
        .freq = 100000,
        .refFreq = 0,
        .SDAport = STHS34PF80_I2C_SDA_PORT,
        .SDApin = STHS34PF80_I2C_SDA_PIN,
        .SCLport = STHS34PF80_I2C_SCL_PORT,
        .SCLpin = STHS34PF80_I2C_SCL_PIN,
    };
    if (I2C_DMA_Init(&i2c_init) != I2CISENABLED) {
        return STHS34PF80_i2cError;
    }

    uint8_t reg_name = STHS34PF80_WHO_AM_I_REG;
    uint8_t chipId = 0;

    I2C_DMA_Read_S i2c_read = {
        .i2c = STHS34PF80_I2C_PORT,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = &reg_name,
        .txCount = 1,
        .rxBuff = &chipId,
        .rxCount = 1
    };

    if (I2C_DMA_Read(&i2c_read) != I2CTRANSFERISSTARTED) {
        return STHS34PF80_i2cError;
    }

    if (chipId != STHS34PF80_DEVICE_ID) {
        return STHS34PF80_noChip;
    }

    sths34pf80_powerDoWn();

    uint8_t reg[2] = { STHS34PF80_LPF1_REG, ((init->Configs->LPF_High << 3) | init->Configs->LPF_Low) };

    I2C_DMA_Write_S i2c_write = {
        .i2c = I2C0,
        .slaveAddress = STHS34PF80_I2C_ADDRESS,
        .txBuff = reg,
        .txCount = 2,
    };

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = STHS34PF80_LPF2_REG;
    reg[1] = ((init->Configs->LPF_Low << 3) | STHS34PF80_ODR_Div_800);

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = STHS34PF80_AVG_TRIM_REG;
    reg[1] = ((init->Configs->T_Average << 4) | init->Configs->TMOS_Average);

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = STHS34PF80_CTRL3_REG;
    reg[1] = ((init->IntConfigs->PolarityLow << 7) | (init->IntConfigs->OpenDrain << 6) | (init->IntConfigs->IntMask << 3) | (init->IntConfigs->Latched << 2)
            | init->IntConfigs->IntCause);

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    reg[0] = ((init->Algorithms->Int_Pulsed << 3) | (init->Algorithms->Comp_Algo << 2) | (init->Algorithms->Presence_AbsTrig << 1));

    if (sths34pf80_write_emb_reg(STHS34PF80_EMB_ALGO_CONFIG_REG, reg, 1) != STHS34PF80_done) {
        return STHS34PF80_fail;
    }

    reg[0] = STHS34PF80_CTRL1_REG;
    reg[1] = ((init->Configs->BlockDataUpdate << 4) | init->Configs->ODR_Frequency);

    if (I2C_DMA_Write(&i2c_write) != I2CTRANSFEREND) {
        return STHS34PF80_i2cError;
    }

    Current_freq = init->Configs->ODR_Frequency;

    return STHS34PF80_done;
}
