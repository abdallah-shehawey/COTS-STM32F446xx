/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    SYSTIC_program.c    >>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SYSTIC                                          **
 **                                                                           **
 **===========================================================================**
 */

#include <stdint.h>
#include "STM32F446xx.h"
#include "ErrTypes.h"

#include "SYSTIC_interface.h"
#include "SYSTIC_private.h"
#include "SYSTIC_config.h"

/**
 * @brief Global variable to store the overflow time to systic
 * @note This variable is used to calculate the number of ticks required for a delay
 */
static float Global_u32SysticOverflow = 0;

/*=================================================================================================================*/
/**
 * @fn SYSTIC_vInit
 * @brief Initialize the SysTick timer
 * @details This function configures:
 *          1. Clock source (AHB or AHB/8)
 *          2. Calculates overflow time based on clock source
 *          3. Configures interrupt state (Enabled/Disabled)
 */
void SYSTIC_vInit(void)
{
  /**
   * @brief Configure clock source and calculate overflow time
   */
#if SYSTIC_CLKSOURCE == CLK_SOURCE_AHB
  /* Use processor clock (AHB) */
  MSYSTIC->CTRL |= (1u << SYSTIC_CTRL_CLKSOURCE);
  Global_u32SysticOverflow = (SYSTIC_MAX_NO_OF_TICKS + 1) * (1.0f / SYSTEM_CLOCK);
#elif SYSTIC_CLKSOURCE == CLK_SOURCE_AHB_DIV8
  /* Use processor clock divided by 8 */
  MSYSTIC->CTRL &= (~(1u << SYSTIC_CTRL_CLKSOURCE));
  Global_u32SysticOverflow = (SYSTIC_MAX_NO_OF_TICKS + 1) * (8.0f / SYSTEM_CLOCK);
#else
#error "Invalid SYSTIC Clock Source"
#endif

  /**
   * @brief Configure interrupt state
   */
#if SYSTIC_TICKINT == ENABLE
  /* Enable SysTick exception generation */
  MSYSTIC->CTRL |= (1u << SYSTIC_CTRL_TICKINT);
#elif SYSTIC_TICKINT == DISABLE
  /* Disable SysTick exception generation */
  MSYSTIC->CTRL &= (~(1u << SYSTIC_CTRL_TICKINT));
#else
#error "Invalid SYSTIC Interrupt Enable"
#endif
}

/**
 * @fn SYSTIC_vDisable
 * @brief Disable the SysTick timer
 * @details This function disables the SysTick timer by clearing the ENABLE bit
 */
void SYSTIC_vDisable(void)
{
  MSYSTIC->CTRL &= (~(1u << SYSTIC_CTRL_ENABLE));
}

/*=================================================================================================================*/
/**
 * @fn SYSTIC_vDelayMs
 * @brief Create a delay in milliseconds using polling method
 * @param Copy_u32MsTime Delay duration in milliseconds
 * @details Function operation:
 *          1. Calculates required number of ticks based on requested time
 *          2. Loads the value and starts the timer
 *          3. Waits for the COUNTFLAG to be set (polling)
 */
void SYSTIC_vDelayMs(uint32_t Copy_u32MsTime)
{
  uint32_t Local_u32Ticks = 0;
  float Local_f32CountOverflow = 0;

  /* Calculate required ticks for millisecond delay */
  Local_f32CountOverflow = (float)(Copy_u32MsTime / (Global_u32SysticOverflow * 1000.0f));
  Local_u32Ticks = (uint32_t)(Local_f32CountOverflow * SYSTIC_MAX_NO_OF_TICKS);

  /* Configure and start timer */
  MSYSTIC->LOAD = Local_u32Ticks;
  MSYSTIC->VAL = 0;
  MSYSTIC->CTRL |= (1u << SYSTIC_CTRL_ENABLE);

  /* Wait until counting is complete */
  while ((MSYSTIC->CTRL & (1u << SYSTIC_CTRL_COUNTFLAG)) == 0)
    ;
}

/*=================================================================================================================*/
/**
 * @fn SYSTIC_vDelayUs
 * @brief Create a delay in microseconds using polling method
 * @param Copy_u32UsTime Delay duration in microseconds
 * @details Function operation:
 *          1. Calculates required number of ticks based on requested time
 *          2. Loads the value and starts the timer
 *          3. Waits for the COUNTFLAG to be set (polling)
 */
void SYSTIC_vDelayUs(uint32_t Copy_u32UsTime)
{
  uint32_t Local_u32Ticks = 0;
  float Local_f32CountOverflow = 0;

  /* Calculate required ticks for microsecond delay */
  Local_f32CountOverflow = (float)(Copy_u32UsTime / (Global_u32SysticOverflow * 1000000.0f));
  Local_u32Ticks = (uint32_t)(Local_f32CountOverflow * SYSTIC_MAX_NO_OF_TICKS);

  /* Configure and start timer */
  MSYSTIC->LOAD = Local_u32Ticks;
  MSYSTIC->VAL = 0;
  MSYSTIC->CTRL |= (1u << SYSTIC_CTRL_ENABLE);

  /* Wait until counting is complete */
  while ((MSYSTIC->CTRL & (1u << SYSTIC_CTRL_COUNTFLAG)) == 0)
    ;
}
/*=================================================================================================================*/
