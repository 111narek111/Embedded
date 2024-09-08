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
#include "pwm.h"
#include <stdio.h>
#include "em_cmu.h"
#include "em_prs.h"

TIMER_Init_TypeDef timerInit =
    {
        .enable = false,
        .debugRun = true,
        .prescale = timerPrescale1,
        .clkSel = timerClkSelHFPerClk,
        .count2x = false,
        .ati = false,
        .fallAction = timerInputActionNone,
        .riseAction = timerInputActionNone,
        .mode = timerModeUp,
        .dmaClrAct = false,
        .quadModeX4 = false,
        .oneShot = false,
        .sync = false
    };

TIMER_InitCC_TypeDef timerCCInit =
    {
        .eventCtrl = timerEventEveryEdge,
        .edge = timerEdgeRising,
        .prsSel = 0,
        .cufoa = timerOutputActionNone,
        .cofoa = timerOutputActionNone,
        .cmoa = timerOutputActionNone,
        .mode = timerCCModePWM,
        .filter = false,
        .prsInput = false,
        .coist = false,
        .outInvert = false,
        .prsOutput = timerPrsOutputDefault,
        .prsInputType = timerPrsInputNone
    };
TIMER_InitCC_TypeDef timerCCInit2 =
    {
        .eventCtrl = timerEventEveryEdge,
        .edge = timerEdgeRising,
        .prsSel = 0,
        .cufoa = timerOutputActionNone,
        .cofoa = timerOutputActionNone,
        .cmoa = timerOutputActionNone,
        .mode = timerCCModePWM,
        .filter = false,
        .prsInput = false,
        .coist = false,
        .outInvert = false,
        .prsOutput = timerPrsOutputLevel,
        .prsInputType = timerPrsInputNone
    };
uint32_t TOP1, TOP2, TOP3, TOP4;
static uint32_t TIMER_TopBufGet(TIMER_TypeDef *timer);
static int PwmChOut(PWM_Channel_Num channel, GPIO_Port_TypeDef port, unsigned int pin);
static uint32_t BrightShow(PWM_Channel_Num channel);
static uint32_t MinBarier(PWM_Struct *pwm);
static int Brightness(PWM_Struct *pwm, uint32_t percent);

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

int PwmChEn(PWM_Channel_Num channel, bool enable)
{
    if (enable)
    {
        switch (channel) {
        case PWM_Channel1:
            CMU_ClockEnable(cmuClock_TIMER0, true);
            TIMER_Init(PWM_TIMER_1, &timerInit);
            TIMER_InitCC(PWM_TIMER_1, PWM_TIMER_1_CC_CH, &timerCCInit);
            TIMER_Enable(PWM_TIMER_1, true);
            return 1;
        case PWM_Channel2:
            CMU_ClockEnable(cmuClock_TIMER1, true);
            TIMER_Init(PWM_TIMER_2, &timerInit);
            TIMER_InitCC(PWM_TIMER_2, PWM_TIMER_2_CC_CH, &timerCCInit);
            TIMER_Enable(PWM_TIMER_2, true);
            return 2;
        case PWM_Channel3:
            CMU_ClockEnable(cmuClock_TIMER2, true);
            TIMER_Init(PWM_TIMER_3, &timerInit);
            TIMER_InitCC(PWM_TIMER_3, PWM_TIMER_3_CC_CH, &timerCCInit);
            TIMER_Enable(PWM_TIMER_3, true);
            return 3;
        case PWM_Channel4:
            CMU_ClockEnable(cmuClock_TIMER3, true);
            TIMER_Init(PWM_TIMER_4, &timerInit);
            TIMER_InitCC(PWM_TIMER_4, PWM_TIMER_4_CC_CH, &timerCCInit2);
            TIMER_Enable(PWM_TIMER_4, true);
            return 4;
        default:
            #ifdef DEBUG
            printf("wrong channel\n");
#endif
            return 0;
        }

    }
    else
    {
        switch (channel) {
        case PWM_Channel1:
            TIMER_Reset(PWM_TIMER_1);
            return 1;
        case PWM_Channel2:
            TIMER_Reset(PWM_TIMER_2);
            return 2;
        case PWM_Channel3:
            TIMER_Reset(PWM_TIMER_3);
            return 3;
        case PWM_Channel4:
            TIMER_Reset(PWM_TIMER_4);
            return 4;
        default:
            #ifdef DEBUG
            printf("wrong channel\n");
#endif
            return 0;
        }
    }
}

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

int FreqSet(PWM_Channel_Num channel, uint32_t frequency)
{
    uint32_t presc = 1;
    uint32_t absolute = PWM_CLK_FREQUENCY / ((frequency * (PWM_MAX_BRIGHT)) / PWM_BRIGHTNESS_STEP);
    uint32_t topValue = 0;
    TIMER_TypeDef *timer;
    while (1)
    {
        if (presc < absolute)
                {
            presc *= 2;
        }
        else if (presc == absolute)
                {
            timerInit.prescale = presc - 1;
            break;
        }
        else
        {
            presc /= 2;
            timerInit.prescale = presc - 1;
            break;
        }

    }

    topValue = (PWM_CLK_FREQUENCY / (presc * frequency)) - 1;

    PwmChEn(channel, true);

    switch (channel) {
    case PWM_Channel1:
        TOP1 = topValue;
        timer = PWM_TIMER_1;
        TIMER_TopBufSet(timer, topValue);
        return 1;
    case PWM_Channel2:
        TOP2 = topValue;
        timer = PWM_TIMER_2;
        TIMER_TopBufSet(timer, topValue);
        return 2;
    case PWM_Channel3:
        TOP3 = topValue;
        timer = PWM_TIMER_3;
        TIMER_TopBufSet(timer, topValue);
        return 3;
    case PWM_Channel4:
        TOP4 = topValue;
        timer = PWM_TIMER_4;
        TIMER_TopBufSet(timer, topValue);
        return 4;
    default:
        #ifdef DEBUG
        printf("wrong channel\n");
#endif
        return 0;
    }
}

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

uint32_t FreqGet(PWM_Channel_Num channel)
{
    uint32_t top = 0;
    uint32_t presc = 0;

    switch (channel) {
    case PWM_Channel1:
        top = TIMER_TopBufGet(PWM_TIMER_1);
        presc = PWM_TIMER_1->CFG & 0xFE0000;
        presc = presc >> PWM_PRESCALER_SHIFT;
        presc += 1;
        break;
    case PWM_Channel2:
        top = TIMER_TopBufGet(PWM_TIMER_2);
        presc = PWM_TIMER_2->CFG & 0xFE0000;
        presc = presc >> PWM_PRESCALER_SHIFT;
        presc += 1;
        break;
    case PWM_Channel3:
        top = TIMER_TopBufGet(PWM_TIMER_3);
        presc = PWM_TIMER_3->CFG & 0xFE0000;
        presc = presc >> PWM_PRESCALER_SHIFT;
        presc += 1;
        break;
    case PWM_Channel4:
        top = TIMER_TopBufGet(PWM_TIMER_4);
        presc = PWM_TIMER_4->CFG & 0xFE0000;
        presc = presc >> PWM_PRESCALER_SHIFT;
        presc += 1;
        break;
    default:
        #ifdef DEBUG
        printf("wrong channel\n");
#endif
        break;
    }
    top += 1;

    uint32_t pwm_freq = PWM_CLK_FREQUENCY / (presc * top);
    return pwm_freq;
}

/******************************************************************************
 * @brief
 *  Connects PWM channel to GPIO.
 *
 * @param[in] channel
 *  PWM channel.
 *
 * @param[in] port
 *  GPIO port to connect to PWM channel.
 *
 * @param[in] pin.
 *  GPIO pin to connect to PWM channel.
 *
 * @return
 *  Function state.
 *  1 if Channel1 GPIO is set.
 *  2 if Channel2 GPIO is set.
 *  3 if Channel3 GPIO is set.
 *  4 if Channel4 GPIO is set.
 *  0 if error.
 *****************************************************************************/

static int PwmChOut(PWM_Channel_Num channel, GPIO_Port_TypeDef port, unsigned int pin)
{
    GPIO_PinModeSet(port, pin, gpioModePushPull, 0);
    switch (channel) {
    case PWM_Channel1:
        GPIO->TIMERROUTE[0].ROUTEEN = ( GPIO->TIMERROUTE[0].ROUTEEN & ~_GPIO_TIMER_ROUTEEN_MASK) | GPIO_TIMER_ROUTEEN_CC0PEN;
        GPIO->TIMERROUTE[0].CC0ROUTE = (GPIO->TIMERROUTE[0].CC0ROUTE & ~_GPIO_TIMER_CC0ROUTE_MASK)
                | ((port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT));
        return 1;
    case PWM_Channel2:
        GPIO->TIMERROUTE[1].ROUTEEN = ( GPIO->TIMERROUTE[1].ROUTEEN & ~_GPIO_TIMER_ROUTEEN_MASK) | GPIO_TIMER_ROUTEEN_CC0PEN;
        GPIO->TIMERROUTE[1].CC0ROUTE = (GPIO->TIMERROUTE[1].CC0ROUTE & ~_GPIO_TIMER_CC0ROUTE_MASK)
                | ((port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT));
        return 2;
    case PWM_Channel3:
        GPIO->TIMERROUTE[2].ROUTEEN = ( GPIO->TIMERROUTE[2].ROUTEEN & ~_GPIO_TIMER_ROUTEEN_MASK) | GPIO_TIMER_ROUTEEN_CC0PEN;
        GPIO->TIMERROUTE[2].CC0ROUTE = (GPIO->TIMERROUTE[2].CC0ROUTE & ~_GPIO_TIMER_CC0ROUTE_MASK)
                | ((port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT) | (pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT));
        return 3;
    case PWM_Channel4:
        CMU_ClockEnable(cmuClock_PRS, true);
        PRS_SourceAsyncSignalSet(PWM_PRS_CHANNEL, PRS_ASYNC_CH_CTRL_SOURCESEL_TIMER3, PRS_ASYNC_CH_CTRL_SIGSEL_TIMER3CC0);
        PRS_PinOutput(PWM_PRS_CHANNEL, prsTypeAsync, port, pin);
        CMU_ClockEnable(cmuClock_PRS, false);
        return 4;
    default:
        #ifdef DEBUG
        printf("wrong channel\n");
#endif
        return 0;
    }
}

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
 * @note
 *  Value must be divided by 10 to get percent.
 *
 * @return
 *  Function state
 *  1 if Channel1 brightness is set.
 *  2 if Channel2 brightness is set.
 *  3 if Channel3 brightness is set.
 *  4 if Channel4 brightness is set.
 *  0 if error.
 *****************************************************************************/

static int Brightness(PWM_Struct *pwm, uint32_t percent)
{

    TIMER_TypeDef *timer;
    uint32_t top;
    uint32_t compare;
    uint32_t minBar;
    uint32_t comp;
    uint32_t topValue;
    switch (pwm->channel) {
    case PWM_Channel1:
        timer = PWM_TIMER_1;
        top = TOP1;
        break;
    case PWM_Channel2:
        timer = PWM_TIMER_2;
        top = TOP2;
        break;
    case PWM_Channel3:
        timer = PWM_TIMER_3;
        top = TOP3;
        break;
    case PWM_Channel4:
        timer = PWM_TIMER_4;
        top = TOP4;
        break;
    default:
        return 0;
    }
    top++;
    if (pwm->frequency > 1000)
            {
        minBar = MinBarier(pwm);
        compare = (top * minBar) / PWM_MAX_BRIGHT;
        if (compare > 1)
                {
            compare--;
        }
        if (pwm->ontime_const != 0)
                {
            if (percent == 0)
                    {
                TIMER_CompareBufSet(timer, 0, 0);
                TIMER_TopBufSet(timer, PWM_TIMER_TOP);
            }
            else if (percent > 0 && percent < minBar)
                    {
                TIMER_CompareBufSet(timer, 0, compare);
                topValue = ((compare + 1) * PWM_MAX_BRIGHT) / percent;
                topValue--;
                TIMER_TopBufSet(timer, topValue);
            }
            else if (percent == minBar)
                    {
                top--;
                TIMER_TopBufSet(timer, top);
                TIMER_CompareBufSet(timer, 0, compare);
            }
            else if (percent > minBar)
                    {
                top--;
                TIMER_TopBufSet(timer, top);
                top++;
                comp = (percent * top) / PWM_MAX_BRIGHT;
                if (comp > 1)
                        {
                    comp--;
                }
                if (comp >= top)
                        {
                    comp = top;
                }
                TIMER_CompareBufSet(timer, 0, comp);
            }
        }
        else
        {
            top--;
            TIMER_TopBufSet(timer, top);
            top++;
            comp = (percent * top) / PWM_MAX_BRIGHT;
            if (comp > 1)
                    {
                comp--;
            }
            if (comp >= top)
                    {
                comp = top;
            }
            TIMER_CompareBufSet(timer, 0, comp);
        }
    }
    else
    {
        top--;
        TIMER_TopBufSet(timer, top);
        top++;
        comp = (percent * top) / PWM_MAX_BRIGHT;
        if (comp > 1)
                {
            comp--;
        }
        if (comp >= top)
                {
            comp = top;
        }
        TIMER_CompareBufSet(timer, 0, comp);
    }

    switch (pwm->channel)
    {
    case Channel1:
        return 1;
    case Channel2:
        return 2;
    case Channel3:
        return 3;
    case Channel4:
        return 4;
    default:
        return 0;
    }

}

/******************************************************************************
 * @brief
 *  Minimal percent value after which to decrease brightness,device will increase
 * frequency instead of duty cycle.
 *
 * @param[in] channel
 *  PWM channel.
 *
 * @return
 *  Minimal percent value.
 *
 * @note
 *  Must be used if you want to decrease brightness by using timer's top value.
 ******************************************************************************/

static uint32_t MinBarier(PWM_Struct *pwm)
{
    if (pwm->ontime_const <= 1.0 || pwm->ontime_const >= 50.0)
            {
        return 0;
    }
    uint32_t ontime = (uint32_t) ((pwm->ontime_const) * 10.0);
    uint32_t minBar = ontime * (pwm->frequency) * PWM_MAX_BRIGHT / 10000000;
    return minBar;
}

/***************************************************************************//**
 * @brief
 *   Get the top buffer value setting for the timer.
 *
 * @param[in] timer
 *   Pointer to the TIMER peripheral register block.
 *
 * @return
 *   Current top buffer value.
 ******************************************************************************/

static uint32_t TIMER_TopBufGet(TIMER_TypeDef *timer)
{
    return timer->TOPB;
}

/******************************************************************************
 *@brief
 *  Get PWM channel brightness.
 *
 *@param[in] channel
 *  PWM channel.
 *
 *@return
 *  PWM channel's Brightness.
 *
 *@note
 * Value must be divided by 10 to get percent.
 *******************************************************************************/

static uint32_t BrightShow(PWM_Channel_Num channel)
{
    uint32_t compare;
    uint32_t brightness;
    uint32_t top;

    switch (channel) {
    case PWM_Channel1:
        compare = PWM_TIMER_1->CC[0].OCB;
        top = TIMER_TopBufGet(PWM_TIMER_1);
        break;
    case PWM_Channel2:
        compare = PWM_TIMER_2->CC[0].OCB;
        top = TIMER_TopBufGet(PWM_TIMER_2);
        break;
    case PWM_Channel3:
        compare = PWM_TIMER_3->CC[0].OCB;
        top = TIMER_TopBufGet(PWM_TIMER_3);
        break;
    case PWM_Channel4:
        compare = PWM_TIMER_4->CC[0].OCB;
        top = TIMER_TopBufGet(PWM_TIMER_4);
        break;
    default:
        #ifdef DEBUG
        printf("wrong channel\n");
#endif
        return 0;
    }
    compare++;
    brightness = (compare * PWM_MAX_BRIGHT) / (top + 1);

    return brightness;
}

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

int PwmBrightnessSet(PWM_Struct *pwm, float percent)
{
    uint32_t bright = (uint32_t) (percent * 10.0);
    return Brightness(pwm, bright);
}

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

float PwmBrightnessGet(PWM_Channel_Num channel)
{
    uint32_t bright = BrightShow(channel);
    float percent = (float) bright / 10.0;
    return percent;
}

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

int PwmChannelInit(PWM_Struct *pwm)
{
    if (pwm->frequency < 400 && pwm->frequency > 20000)
            {
        return 0;
    }
    FreqSet(pwm->channel, pwm->frequency);
    PwmChOut(pwm->channel, pwm->port, pwm->pin);
    Brightness(pwm, 0);

    switch (pwm->channel)
    {
    case Channel1:
        return 1;
    case Channel2:
        return 2;
    case Channel3:
        return 3;
    case Channel4:
        return 4;
    default:
        return 0;
    }
}

