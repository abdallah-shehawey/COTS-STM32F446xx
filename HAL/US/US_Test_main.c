/**
 **===========================================================================**
 **                     US_Test_main.c — Ultrasonic Test                      **
 **                                                                           **
 **   Author : Abdallah Saleh                                                 **
 **   MCU    : STM32F446RE (Nucleo-64)                                        **
 **   Brief  : Trigger Buzzer when distance is less than 5cm                  **
 **                                                                           **
 **   Wiring:                                                                  **
 **     Ultrasonic Sensor (US):                                               **
 **       VCC  -> 5V                                                           **
 **       GND  -> GND                                                          **
 **       TRIG -> PA0                                                          **
 **       ECHO -> PA1 (Use voltage divider if needed: 5V signals to 3.3V)      **
 **                                                                           **
 **     Buzzer (Active High):                                                  **
 **       + -> PB0                                                            **
 **       - -> GND                                                            **
 **                                                                           **
 **===========================================================================**
 */

#include <stdint.h>

/* LIB Includes */
#include "STM32F446xx.h"
#include "STD_MACROS.h"
#include "ErrTypes.h"

/* MCAL Includes */
#include "RCC_interface.h"
#include "GPIO_interface.h"
#include "TIM_interface.h"

/* HAL Includes */
#include "US_interface.h"
#include "BUZ_interface.h"

/*_______________________________________________________________________________*/
/*                         Sensor & Buzzer Configuration                         */
/*_______________________________________________________________________________*/

/**
 * @brief Select ICU timer & channel for Ultrasonic ECHO (PA1 -> TIM2_CH2).
 *        Delay uses Basic Timer TIM_TIMER7 to avoid conflicting with:
 *          - TIM_TIMER2 (US ICU)
 *          - TIM_TIMER5 (CLCDT driver)
 *          - TIM_TIMER6 (MPU9250 delay driver)
 */
#define US_ICU_TIMER     TIM_TIMER2
#define US_ECHO_CHANNEL  TIM_CHANNEL2
#define US_DELAY_TIMER   TIM_TIMER7

/* Pin Definitions */
#define US_TRIG_PORT     GPIO_PORTA
#define US_TRIG_PIN      GPIO_PIN0
#define US_ECHO_PORT     GPIO_PORTA
#define US_ECHO_PIN      GPIO_PIN1

#define BUZ_PORT         GPIO_PORTB
#define BUZ_PIN          GPIO_PIN0

/* Configuration Constants */
#define ALERT_DISTANCE_CM    5U
#define MEASURE_DELAY_MS     60U

/*_______________________________________________________________________________*/
/*                              Helper: Enable RCC                               */
/*_______________________________________________________________________________*/

/**
 * @brief Enable the RCC clock for a given timer using the MCAL RCC driver.
 */
static void App_vEnableTimerClock(TIM_Num_t Copy_eTimer)
{
    switch (Copy_eTimer)
    {
        case TIM_TIMER2: RCC_enumABPPerSts(RCC_APB1, RCC_TIM2EN, RCC_PER_ON); break;
        case TIM_TIMER3: RCC_enumABPPerSts(RCC_APB1, RCC_TIM3EN, RCC_PER_ON); break;
        case TIM_TIMER4: RCC_enumABPPerSts(RCC_APB1, RCC_TIM4EN, RCC_PER_ON); break;
        case TIM_TIMER5: RCC_enumABPPerSts(RCC_APB1, RCC_TIM5EN, RCC_PER_ON); break;
        case TIM_TIMER6: RCC_enumABPPerSts(RCC_APB1, RCC_TIM6EN, RCC_PER_ON); break;
        case TIM_TIMER7: RCC_enumABPPerSts(RCC_APB1, RCC_TIM7EN, RCC_PER_ON); break;
        case TIM_TIMER1: RCC_enumABPPerSts(RCC_APB2, RCC_TIM1EN, RCC_PER_ON); break;
        case TIM_TIMER8: RCC_enumABPPerSts(RCC_APB2, RCC_TIM8EN, RCC_PER_ON); break;
        default: break;
    }
}

/**
 * @brief Enable the RCC clock for the selected timers and GPIO ports.
 */
static void App_vEnableClocks(void)
{
    /* Enable GPIOA and GPIOB clocks via MCAL RCC driver */
    RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOAEN, RCC_PER_ON);
    RCC_enumAHPPerSts(RCC_AHB1, RCC_GPIOBEN, RCC_PER_ON);

    /* Enable Ultrasonic ICU timer and separate Delay timer clocks */
    App_vEnableTimerClock(US_ICU_TIMER);
    App_vEnableTimerClock(US_DELAY_TIMER);
}

/*_______________________________________________________________________________*/
/*                                    Main                                       */
/*_______________________________________________________________________________*/

int main(void)
{
    /* Ensure ICU and Delay never share the same timer */
    (void)sizeof(char[(US_ICU_TIMER != US_DELAY_TIMER) ? 1 : -1]);

    /* 1. Initialize Peripheral Clocks */
    App_vEnableClocks();

    /* 2. Configure and Initialize Ultrasonic Sensor */
    US_Config_t Sensor = {
        .Timer    = US_ICU_TIMER,
        .Channel  = US_ECHO_CHANNEL,
        .TrigPort = US_TRIG_PORT,
        .TrigPin  = US_TRIG_PIN,
        .EchoPort = US_ECHO_PORT,
        .EchoPin  = US_ECHO_PIN
    };
    US_vInit(&Sensor);

    /* 3. Initialize Dedicated Delay Timer (1kHz tick at 16MHz HSI) */
    TIM_Config_t DelayTimerCfg = {
        .Timer           = US_DELAY_TIMER,
        .Prescaler       = 16000U - 1U,
        .AutoReloadValue = MEASURE_DELAY_MS,
        .Mode            = TIM_COUNTERMODE_UP
    };
    TIM_vInit(&DelayTimerCfg);

    /* 4. Configure and Initialize Buzzer */
    BUZ_Config_t Buzzer = {
        .Port        = BUZ_PORT,
        .Pin         = BUZ_PIN,
        .ActiveState = ACTIVE_HIGH
    };
    BUZ_Init(&Buzzer);

    uint16_t     Local_u16Distance_cm = 0U;
    ErrorState_t Local_Status         = OK;

    while (1)
    {
        /* Perform Distance Measurement */
        Local_Status = US_u16ReadDistance_cm(&Sensor, &Local_u16Distance_cm);

        if (Local_Status == OK)
        {
            /* Check if an object is within the alert threshold */
            if (Local_u16Distance_cm < ALERT_DISTANCE_CM)
            {
                BUZ_On(&Buzzer);    /* Distance < 5cm -> Alarm ON  */
            }
            else
            {
                BUZ_Off(&Buzzer);   /* Distance >= 5cm -> Alarm OFF */
            }
        }
        else if (Local_Status == TIMEOUT_STATE)
        {
            /* No object detected within timeout range -> Ensure Buzzer is OFF */
            BUZ_Off(&Buzzer);
        }

        /* 60ms delay between triggers on dedicated Basic Timer TIM7 */
        TIM_vDelayMs(US_DELAY_TIMER, MEASURE_DELAY_MS);
    }

    return 0;
}
