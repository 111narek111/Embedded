/*******************************************************************************
 Lynxal Inc.

 CONFIDENTIAL AND PROPRIETARY
 FOR USE BY AUTHORIZED PERSONS ONLY

 This is an unpublished work fully protected by the copyright
 laws and is a trade secret belonging to the copyright holder.

 Copyright (c) 2021-2023 Lynxal Inc. All Rights Reserved.
 ********************************************************************************
 sths34pf80_defs.h OPT4048 Light sensor registers definitions
 *******************************************************************************/

#ifndef STHS34PF80_DEFS_H_
#define STHS34PF80_DEFS_H_

#define STHS34PF80_LPF1_REG                 0x0C
#define STHS34PF80_LPF2_REG                 0x0D
#define STHS34PF80_WHO_AM_I_REG             0x0F
#define STHS34PF80_AVG_TRIM_REG             0x10
#define STHS34PF80_CTRL0_REG                0x17
#define STHS34PF80_SENS_DATA_REG            0x1D
#define STHS34PF80_CTRL1_REG                0x20
#define STHS34PF80_CTRL2_REG                0x21
#define STHS34PF80_CTRL3_REG                0x22
#define STHS34PF80_STATUS_REG               0x23
#define STHS34PF80_FUNC_STATUS_REG          0x25
#define STHS34PF80_TOBJECT_L_REG            0x26
#define STHS34PF80_TOBJECT_H_REG            0x27
#define STHS34PF80_TAMBIENT_L_REG           0x28
#define STHS34PF80_TAMBIENT_H_REG           0x29
#define STHS34PF80_TOBJ_COMP_L_REG          0x38
#define STHS34PF80_TOBJ_COMP_H_REG          0x39
#define STHS34PF80_TPRESENCE_L_REG          0x3A
#define STHS34PF80_TPRESENCE_H_REG          0x3B
#define STHS34PF80_TMOTION_L_REG            0x3C
#define STHS34PF80_TMOTION_H_REG            0x3D
#define STHS34PF80_TAMB_SHOCK_L_REG         0x3E
#define STHS34PF80_TAMB_SHOCK_H_REG         0x3F

#define STHS34PF80_EMB_FUNC_CFG_ADDR_REG    0x08
#define STHS34PF80_EMB_FUNC_CFG_DATA_REG    0x09
#define STHS34PF80_EMB_PAGE_RW_REG          0x11
#define STHS34PF80_EMB_PRESENCE_THS_REG1    0x20
#define STHS34PF80_EMB_PRESENCE_THS_REG2    0x21
#define STHS34PF80_EMB_MOTION_THS_REG1      0x22
#define STHS34PF80_EMB_MOTION_THS_REG2      0x23
#define STHS34PF80_EMB_TAMB_SHOCK_THS_REG1  0x24
#define STHS34PF80_EMB_TAMB_SHOCK_THS_REG2  0x25
#define STHS34PF80_EMB_HYST_MOTION_REG      0x26
#define STHS34PF80_EMB_HYST_PRESENCE_REG    0x27
#define STHS34PF80_EMB_ALGO_CONFIG_REG      0x28
#define STHS34PF80_EMB_HYST_TAMB_SHOCK_REG  0x29
#define STHS34PF80_EMB_RESET_ALGO_REG       0x2A

#define STHS34PF80_DEVICE_ID                0xD3

#define STHS34PF80_EMB_RESET_ALGO               1
#define STHS34PF80_CTRL1_SET_POWER_DOWN         0
#define STHS34PF80_CTRL2_TRIGGER_ONE_SHOT       1
#define STHS34PF80_CTRL2_REBOOT                 (1 << 7)

#define STHS34PF80_I2C_ADDRESS                  0x5A

#define STHS34PF80_I2C_PORT        I2C0
#define STHS34PF80_I2C_SDA_PORT    gpioPortC
#define STHS34PF80_I2C_SDA_PIN     5
#define STHS34PF80_I2C_SCL_PORT    gpioPortC
#define STHS34PF80_I2C_SCL_PIN     6

#define STHS34PF80_THRESHOLD_MAX   750
#define STHS34PF80_THRESHOLD_MIN   90


#endif /* STHS34PF80_DEFS_H_ */
