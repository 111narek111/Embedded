/***************************************************************************//**
 * @file
 * @brief Top level application functions for PWM driver
 *******************************************************************************
 * Description
 *
 * This file contains the top-level functions for configuring and controlling
 * Pulse Width Modulation (PWM) on the Silicon Labs devices. It
 * provides initialization routines, frequency and duty cycle control, and
 * other utility functions to manage PWM channels.
 *
 * Author
 * Narek Hakobyan
 ******************************************************************************/
#ifndef PWM_H
#define PWM_H
#include "em_device.h"
#include "em_gpio.h"
#include "em_timer.h"

typedef enum {
    PWM_Channel1 = 1, /*for A,B,C,D ports*/
    PWM_Channel2 = 2, /*for A,B,C,D ports*/
    PWM_Channel3 = 3, /*for A,B ports*/
    PWM_Channel4 = 4 /*for C,D ports*/
} PWM_Channel_Num;

typedef struct {
    /**PWM Channel */
    PWM_Channel_Num channel;
    /**PWM output port */
    GPIO_Port_TypeDef port;
    /**PWM output pin */
    unsigned int pin;
    /**PWM frequency.(Set this value between 400 and 20000) */
    uint32_t frequency;
    /**Value below which PWM will change by changing frequency
     * value is in microseconds.(don't set this value equal or below 1 or higher/equal 50).
     * Set it zero if you don't need on-time constant change of brightness. */
    float ontime_const;
} PWM_Struct;

#define PWM_TIMER_1 TIMER0   // 32 bit
#define PWM_TIMER_2 TIMER1   //16 bit
#define PWM_TIMER_3 TIMER2   //16 bit
#define PWM_TIMER_4 TIMER3   //16 bit
#define PWM_TIMER_1_CC_CH    0
#define PWM_TIMER_2_CC_CH    0
#define PWM_TIMER_3_CC_CH    0
#define PWM_TIMER_4_CC_CH    0

#define PWM_PRESCALER_SHIFT  18
#define PWM_PRS_CHANNEL      5

#ifndef PWM_CLK_FREQUENCY
#define PWM_CLK_FREQUENCY 80000000LU /*hz*/
#endif

#ifndef PWM_TIMER_TOP
#define PWM_TIMER_TOP 65535LU  //16 bit timer's top value
#endif

#define PWM_MIN_FREQ   400LU
#define PWM_MAX_FREQ   20000LU
#define PWM_MAX_BRIGHT 1000LU


#define PWM_BRIGHTNESS_STEP  1LU

/******************************************************************************
 * @brief
 *   Enables chosen PWM channel.
 *
 * @param[in] channel
 *   PWM channel.
 *
 * @param[in] enable
 *   Set to true to enable PWM channel,otherwise false.
 *
 * @return
 * Function state
 *  1 if Channel1 is enabled.
 *  2 if Channel2 is enabled.
 *  3 if Channel3 is enabled.
 *  4 if Channel4 is enabled.
 *  0 if error.
 *****************************************************************************/

int PwmChEn(PWM_Channel_Num channel, bool enable);

/******************************************************************************
 * @brief
 *  PWM channel frequency set.
 *
 * @param[in] channel
 *  PWM channel.
 *
 * @param[in] frequency
 *  Frequency to set on channel.
 *
 * @return
 * Function state
 *  1 if Channel1 frequency is set.
 *  2 if Channel2 frequency is set.
 *  3 if Channel3 frequency is set.
 *  4 if Channel4 frequency is set.
 *  0 if error.
 *****************************************************************************/

int FreqSet(PWM_Channel_Num channel, uint32_t frequency);

/******************************************************************************
 * @brief
 *   Get PWM channel frequency.
 *
 * @param[in] channel
 *   PWM channel.
 *
 * @return
 *   PWM channel's frequency.
 ******************************************************************************/

uint32_t FreqGet(PWM_Channel_Num channel);

/******************************************************************************
 * @brief
 *  Initialize PWM Channel and Set it on 0% brightness.
 *
 * @param[in] pwm
 *  Struct with PWM channel parameters.
 *
 * @return
 *  Function state
 *  1 if Channel1 brightness is changed.
 *  2 if Channel2 brightness is changed.
 *  3 if Channel3 brightness is changed.
 *  4 if Channel4 brightness is changed.
 *  0 if error.
 ******************************************************************************/

int PwmChannelInit(PWM_Struct *pwm);

/*****************************************************************************
 * @brief
 *  PWM channel brightness set.
 *
 * @param[in] channel
 * PWM channel.
 *
 * @param[in] percent
 * Brightness percentage.
 *
 * @return
 *  Function state
 *  1 if Channel1 brightness is set.
 *  2 if Channel2 brightness is set.
 *  3 if Channel3 brightness is set.
 *  4 if Channel4 brightness is set.
 *  0 if error.
 *****************************************************************************/

int PwmBrightnessSet(PWM_Struct *pwm, float percent);

/******************************************************************************
 *@brief
 *  Get PWM channel brightness.
 *
 *@param[in] channel
 *  PWM channel.
 *
 *@return
 *  PWM channel's Brightness.
 *******************************************************************************/

float PwmBrightnessGet(PWM_Channel_Num channel);
#endif  // PWM_H
