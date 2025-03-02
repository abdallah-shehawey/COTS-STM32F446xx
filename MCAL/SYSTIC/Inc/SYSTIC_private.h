/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    SYSTIC_private.h    >>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SYSTIC                                          **
 **                                                                           **
 **===========================================================================**
 */

#ifndef MCAL_SYSTIC_PRIVATE_H_
#define MCAL_SYSTIC_PRIVATE_H_

/**
 * @brief SysTick Control and Status Register bit positions
 * @note These bits control the SysTick timer operation and show its status
 */
#define SYSTIC_CTRL_ENABLE (0U)     /* Counter Enable */
#define SYSTIC_CTRL_TICKINT (1U)    /* SysTick Exception Request Enable */
#define SYSTIC_CTRL_CLKSOURCE (2U)  /* Clock Source Selection */
#define SYSTIC_CTRL_COUNTFLAG (16U) /* Timer Count Flag */

/**
 * @brief Maximum number of ticks the SysTick timer can count
 * @note SysTick is a 24-bit counter, so max value is 2^24 - 1
 */
#define SYSTIC_MAX_NO_OF_TICKS (0x00FFFFFF)

/**
 * @brief Clock source selection values
 */
#define CLK_SOURCE_AHB (1U)      /* Use processor clock (AHB) */
#define CLK_SOURCE_AHB_DIV8 (0U) /* Use processor clock/8 */

/**
 * @brief Enable/Disable definitions
 */
#define ENABLE (1U)
#define DISABLE (0U)

#endif /* MCAL_SYSTIC_PRIVATE_H_ */