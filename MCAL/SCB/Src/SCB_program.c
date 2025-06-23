/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    SCB_program.c    >>>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : SCB                                             **
 **                                                                           **
 **===========================================================================**
 */
#include <stdint.h>

#include "ErrTypes.h"

#include "../Inc/SCB_interface.h"
#include "../Inc/SCB_private.h"
#include "../Inc/SCB_config.h"

#include "STM32F446xx.h"

ErrorState_t SCB_vSetPriorityGrouping(uint8_t Copy_u8PriorityGrouping)
{
  uint8_t local_u8ErrorState = OK;
  if (Copy_u8PriorityGrouping > SCB_MAX_PRIORITY_GROUPING)
  {
    local_u8ErrorState = NOK;
  }
  else
  {
    MSCB->AIRCR = AIRCR_VECTKEY | (Copy_u8PriorityGrouping << 8);
  }
  return local_u8ErrorState;
}
