/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<    SYSTIC_interface.h    >>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SYSTIC                                          **
 **                                                                           **
 **===========================================================================**
 */

#ifndef MCAL_SYSTIC_INTERFACE_H_
#define MCAL_SYSTIC_INTERFACE_H_

#include <stdint.h>
#include "ErrTypes.h"

/**
 * @fn SYSTIC_vInit
 * @brief Initialize the SysTick timer with configured settings
 * @details This function initializes the SysTick timer with the clock source
 *          and interrupt settings defined in the configuration file
 */
void SYSTIC_vInit(void);

/**
 * @fn SYSTIC_vDisable
 * @brief Disable the SysTick timer
 * @details This function disables the SysTick timer by clearing the ENABLE bit
 */
void SYSTIC_vDisable(void);

/**
 * @fn SYSTIC_vDelayMs
 * @brief Generate a delay in milliseconds using polling method
 * @param Copy_u32Time Time in milliseconds to delay
 * @note This function blocks until the delay is complete
 */
void SYSTIC_vDelayMs(uint32_t Copy_u32Time);

/**
 * @fn SYSTIC_vDelayUs
 * @brief Generate a delay in microseconds using polling method
 * @param Copy_u32Time Time in microseconds to delay
 * @note This function blocks until the delay is complete
 */
void SYSTIC_vDelayUs(uint32_t Copy_u32Time);

#endif /* MCAL_SYSTIC_INTERFACE_H_ */