/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    SYSTIC_config.h    >>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SYSTIC                                          **
 **                                                                           **
 **===========================================================================**
 */

#ifndef MCAL_SYSTIC_CONFIG_H_
#define MCAL_SYSTIC_CONFIG_H_

/**
 * @brief System clock frequency in Hz
 * @note This should match your MCU's configured system clock
 */
#define SYSTEM_CLOCK 16000000UL /* 16 MHz */

/**
 * @brief SysTick Clock Source Configuration
 * @note Options:
 *       - CLK_SOURCE_AHB: Use processor clock
 *       - CLK_SOURCE_AHB_DIV8: Use processor clock divided by 8
 */
#define SYSTIC_CLKSOURCE CLK_SOURCE_AHB

/**
 * @brief SysTick Interrupt Configuration
 * @note Options:
 *       - ENABLE: Enable SysTick exception
 *       - DISABLE: Disable SysTick exception
 */
#define SYSTIC_TICKINT DISABLE

#endif /* MCAL_SYSTIC_CONFIG_H_ */