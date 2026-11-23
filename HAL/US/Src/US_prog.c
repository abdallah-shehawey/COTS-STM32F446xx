/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<      US_prog.c        >>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Saleh                                  **
 **                  Layer  : HAL                                             **
 **                  SWC    : US (Ultrasonic Distance Sensor - ICU Mode)      **
 **===========================================================================**
 */

#include <stdint.h>
#include "STM32F446xx.h"
#include "STD_MACROS.h"
#include "ErrTypes.h"
#include "GPIO_interface.h"
#include "TIM_interface.h"
#include "TIM_private.h"
#include "US_interface.h"
#include "US_config.h"
#include "US_private.h"

static TIM_TypeDef *US_TIM_Array[TIM_TIMER_COUNT] = {
    TIM2, TIM3, TIM4, TIM5, TIM1, TIM8, TIM6, TIM7
};

/**************************************         Private Functions
 * ******************************************/

static void US_vSendTrigger(const US_Config_t *pxSensor)
{
    GPIO_enumWritePinVal(pxSensor->TrigPort, pxSensor->TrigPin, GPIO_PIN_LOW);
    for(volatile uint32_t i = 0; i < 1000; i++); 

    GPIO_enumWritePinVal(pxSensor->TrigPort, pxSensor->TrigPin, GPIO_PIN_HIGH);
    for(volatile uint32_t i = 0; i < 500; i++); /* 15-20us */

    GPIO_enumWritePinVal(pxSensor->TrigPort, pxSensor->TrigPin, GPIO_PIN_LOW);
}

/**************************************         Public Functions
 * ******************************************/

ErrorState_t US_vInit(const US_Config_t *pxSensor)
{
    if (pxSensor == NULL) return NULL_POINTER;
    if (pxSensor->Timer >= TIM_TIMER6 || pxSensor->Channel > TIM_CHANNEL4) return NOK;

    /* 1. Initialize GPIO Pins */
    GPIO_PinConfig_t TrigCfg = {
        .Port = pxSensor->TrigPort, .PinNum = pxSensor->TrigPin,
        .Mode = GPIO_OUTPUT, .Otype = GPIO_PUSH_PULL,
        .Speed = GPIO_MEDIUM_SPEED, .PullType = GPIO_NO_PULL
    };
    GPIO_enumPinInit(&TrigCfg);

    GPIO_PinConfig_t EchoCfg = {
        .Port = pxSensor->EchoPort, .PinNum = pxSensor->EchoPin,
        .Mode = GPIO_ALTFN, .Otype = GPIO_PUSH_PULL,
        .Speed = GPIO_VERY_HIGH_SPEED, .PullType = GPIO_NO_PULL
    };

    if (pxSensor->Timer == TIM_TIMER1 || pxSensor->Timer == TIM_TIMER2) EchoCfg.AlternateFunction = GPIO_AF1;
    else if (pxSensor->Timer >= TIM_TIMER3 && pxSensor->Timer <= TIM_TIMER5) EchoCfg.AlternateFunction = GPIO_AF2;
    else if (pxSensor->Timer == TIM_TIMER8) EchoCfg.AlternateFunction = GPIO_AF3;
    else return NOK;
    GPIO_enumPinInit(&EchoCfg);

    /* 2. Configure Timer Prescaler for 1us resolution via MCAL TIM driver */
    TIM_TypeDef *TIMx = US_TIM_Array[pxSensor->Timer];
    uint16_t Local_u16PSC = (uint16_t)((US_SYS_CLK_HZ / 1000000U) - 1U);
    uint32_t Local_u32ARR = (pxSensor->Timer == TIM_TIMER2 || pxSensor->Timer == TIM_TIMER5) ? 0xFFFFFFFFUL : 0xFFFFUL;

    TIM_Config_t TIM_Cfg = {
        .Timer           = pxSensor->Timer,
        .Prescaler       = Local_u16PSC,
        .AutoReloadValue = Local_u32ARR,
        .Mode            = TIM_COUNTERMODE_UP
    };
    TIM_vInit(&TIM_Cfg);

    /* 3. HARDWARE SPECIFIC: Enable Advanced Timer Output Logic */
    if (pxSensor->Timer == TIM_TIMER1 || pxSensor->Timer == TIM_TIMER8) {
        SET_BIT(TIMx->BDTR, TIM_BDTR_MOE);
    }

    TIMx->SR = 0;

    /* 4. Configure ICU Polarity */
    TIM_ICConfig_t IC_Cfg = {
        .Timer = pxSensor->Timer, .Channel = pxSensor->Channel,
        .Selection = TIM_IC_SELECTION_DIRECT_TI, .Prescaler = TIM_IC_PSC_DIV1,
        .Polarity = TIM_POLARITY_HIGH, .Filter = 2 /* Minimal filter for stability */
    };
    TIM_vIC_Init(&IC_Cfg);

    TIM_vStart(pxSensor->Timer);

    return OK;
}

ErrorState_t US_u16ReadDistance_cm(const US_Config_t *pxSensor, uint16_t *pu16Dist_cm)
{
    uint32_t t1 = 0, t2 = 0, high_ticks = 0;
    uint32_t Local_u32Timeout = 1000000; 
    
    if (pxSensor == NULL || pu16Dist_cm == NULL) return NULL_POINTER;
    if (pxSensor->Timer >= TIM_TIMER6 || pxSensor->Channel > TIM_CHANNEL4) return NOK;

    TIM_TypeDef *TIMx = US_TIM_Array[pxSensor->Timer];
    uint32_t CCxIF_Mask = (1UL << (pxSensor->Channel + 1));

    /* 1. Reset State to Rising Edge */
    TIM_vSetICPolarity(pxSensor->Timer, pxSensor->Channel, TIM_POLARITY_HIGH);
    TIMx->SR = 0; 
    TIMx->CNT = 0; /* Reset count to 0 for 16-bit safety */
    
    /* 2. Send Trigger */
    US_vSendTrigger(pxSensor);

    /* 3. Wait for Rising Edge */
    while (!((TIMx->SR) & CCxIF_Mask) && --Local_u32Timeout);
    if (Local_u32Timeout == 0) return TIMEOUT_STATE;
    TIM_u32GetCaptureValue(pxSensor->Timer, pxSensor->Channel, &t1);

    /* Special 16-bit handling: Reset count after first capture to prevent wrap-around */
    if (pxSensor->Timer == TIM_TIMER3 || pxSensor->Timer == TIM_TIMER4) {
        TIMx->CNT = 0; 
        t1 = 0; 
    }

    /* 4. Switch to Falling Edge */
    TIM_vSetICPolarity(pxSensor->Timer, pxSensor->Channel, TIM_POLARITY_LOW);
    TIMx->SR = 0; 
    
    /* 5. Wait for Falling Edge */
    Local_u32Timeout = 1000000;
    while (!((TIMx->SR) & CCxIF_Mask) && --Local_u32Timeout);
    if (Local_u32Timeout == 0) return TIMEOUT_STATE;
    TIM_u32GetCaptureValue(pxSensor->Timer, pxSensor->Channel, &t2);

    /* 6. Distance Calculation */
    if (t2 >= t1) high_ticks = t2 - t1;
    else {
        uint32_t MaxVal = (pxSensor->Timer == TIM_TIMER2 || pxSensor->Timer == TIM_TIMER5) ? 0xFFFFFFFF : 0xFFFF;
        high_ticks = (MaxVal - t1) + t2 + 1;
    }

    *pu16Dist_cm = (uint16_t)(high_ticks / US_SOUND_SPEED_FACTOR);

    /* Cleanup */
    TIM_vSetICPolarity(pxSensor->Timer, pxSensor->Channel, TIM_POLARITY_HIGH);

    return OK;
}
