/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    RCC_program.c    >>>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : RCC                                             **
 **                                                                           **
 **===========================================================================**
 */

#include <ErrTypes.h>
#include "STD_MACROS.h"

#include <stdint.h>

#include "stm32f446xx.h"
#include "RCC_config.h"
#include "RCC_interface.h"
#include "RCC_private.h"

/* Global variable to track RCC driver state (IDLE/BUSY) */
static uint8_t RCC_u8State = IDLE;

/**
 * @brief   Enable or disable a specific clock source (HSI, HSE, PLL)
 * @param   Copy_u8CLK: Clock source (HSI_CLK, HSE_CLK, PLL_CLK)
 * @param   Copy_u8Status: Clock status (CLK_ON, CLK_OFF)
 * @return  RCC_ErrorState: Error status (RCC_OK, TIMEOUT_STATE, BUSY_STATE)
 */
RCC_ErrorState RCC_enumSetClkSts(uint8_t Copy_u8CLK, uint8_t Copy_u8Status)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;
  uint32_t Local_u32TimeoutCounter = 0;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    if (Copy_u8CLK == HSI_CLK)
    {
      /* HSI Clock Control */
      if (Copy_u8Status == CLK_ON)
      {
        /* Enable HSI and wait for it to be ready */
        SET_BIT(MRCC->CR, 0);
        while (READ_BIT(MRCC->CR, 1) == 0 && Local_u32TimeoutCounter != RCC_u32TIMEOUT)
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == RCC_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
      }
      else if (Copy_u8Status == CLK_OFF)
      {

        CLR_BIT(MRCC->CR, 0);
        while (READ_BIT(MRCC->CR, 1) == 1 && Local_u32TimeoutCounter != RCC_u32TIMEOUT)
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == RCC_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
      }
      else
      {
      }
    }
    else if (Copy_u8CLK == HSE_CLK)
    {
      /* HSE Clock Control */
      if (Copy_u8Status == CLK_ON)
      {
        /* Enable HSE bypass and HSE clock, then wait for it to be ready */
        SET_BIT(MRCC->CR, 18); /* Enable HSE bypass */
        SET_BIT(MRCC->CR, 16); /* Enable HSE clock */
        while (READ_BIT(MRCC->CR, 17) == 0 && Local_u32TimeoutCounter != RCC_u32TIMEOUT)
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == RCC_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
      }

      else if (Copy_u8Status == CLK_OFF)
      {
        /* Disable HSE clock and wait for it to stop */
        CLR_BIT(MRCC->CR, 16);
        while (READ_BIT(MRCC->CR, 17) == 1 && Local_u32TimeoutCounter != RCC_u32TIMEOUT)
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == RCC_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
      }
    }
    else if (Copy_u8CLK == PLL_CLK)
    {
      /* PLL Clock Control */
      if (Copy_u8Status == CLK_ON)
      {
        /* Enable PLL and wait for it to be ready */
        SET_BIT(MRCC->CR, 24);
        while (READ_BIT(MRCC->CR, 25) == 0 && Local_u32TimeoutCounter != RCC_u32TIMEOUT)
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == RCC_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
      }

      else if (Copy_u8Status == CLK_OFF)
      {
        /* Disable PLL and wait for it to stop */
        CLR_BIT(MRCC->CR, 24);
        while (READ_BIT(MRCC->CR, 25) == 1 && Local_u32TimeoutCounter != RCC_u32TIMEOUT)
        {
          Local_u32TimeoutCounter++;
        }

        if (Local_u32TimeoutCounter == RCC_u32TIMEOUT)
        {
          Local_u8ErrorState = TIMEOUT_STATE;
        }
      }
    }
    else
    {
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Set the system clock source
 * @param   Copy_u8CLK: Clock source to be used as system clock
 *                      (HSI_CLK, HSE_CLK, PLLP_CLK, PLLR_CLK)
 * @return  RCC_ErrorState: Error status (RCC_OK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumSetSysClk(uint8_t Copy_u8CLK)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    switch (Copy_u8CLK)
    {
    case HSI_CLK:
      /* Select HSI as system clock */
      MRCC->CFGR &= ~(SYS_CLK_MASK);
      MRCC->CFGR |= Copy_u8CLK;
      break;

    case HSE_CLK:
      /* Select HSE as system clock */
      MRCC->CFGR &= ~(SYS_CLK_MASK);
      MRCC->CFGR |= Copy_u8CLK;
      break;

    case PLLP_CLK:
      /* Select PLL P output as system clock */
      MRCC->CFGR &= ~(SYS_CLK_MASK);
      MRCC->CFGR |= Copy_u8CLK;
      break;

    case PLLR_CLK:
      /* Select PLL R output as system clock */
      MRCC->CFGR &= ~(SYS_CLK_MASK);
      MRCC->CFGR |= Copy_u8CLK;
      break;
    default:
      break;
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Configure PLL parameters including source, dividers and multiplier
 * @param   Copy_PLLConfig: Pointer to PLL configuration structure
 * @return  RCC_ErrorState: Error status (RCC_OK, NOK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumPLLConfig(const RCC_PLLConfig_t *Copy_PLLConfig)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    /* Validate configuration parameters */
    if ((Copy_PLLConfig != NULL) &&
        (Copy_PLLConfig->PLLM_Div >= MPLL_DIV_2 && Copy_PLLConfig->PLLM_Div <= MPLL_DIV_63) &&
        (Copy_PLLConfig->PLLN_Mult >= NPLL_CLK_MULT_50 && Copy_PLLConfig->PLLN_Mult <= NPLL_CLK_MULT_432) &&
        (Copy_PLLConfig->PLLSource == PLL_HSI || Copy_PLLConfig->PLLSource == PLL_HSE) &&
        (Copy_PLLConfig->PLLP_Div >= PPLL_DIV_2 && Copy_PLLConfig->PLLP_Div <= PPLL_DIV_8))
    {
      /* Configure PLL source */
      if (Copy_PLLConfig->PLLSource == PLL_HSI)
      {
        CLR_BIT(MRCC->PLLCFGR, 22);
      }
      else if (Copy_PLLConfig->PLLSource == PLL_HSE)
      {
        SET_BIT(MRCC->PLLCFGR, 22);
      }

      /* Configure PLL M division factor */
      MRCC->PLLCFGR &= (~(MPLL_DIV_MASK));
      MRCC->PLLCFGR |= (Copy_PLLConfig->PLLM_Div);

      /* Configure PLL N multiplication factor */
      MRCC->PLLCFGR &= (~(NPLL_MULT_MASK << 6));
      MRCC->PLLCFGR |= (Copy_PLLConfig->PLLN_Mult << 6);

      /* Configure PLL P division factor */
      MRCC->PLLCFGR &= (~(PPLL_DIV_MASK << 16));
      MRCC->PLLCFGR |= ((Copy_PLLConfig->PLLP_Div * 2 + 2) << 16);
    }
    else
    {
      Local_u8ErrorState = NOK;
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Configure AHB bus clock prescaler
 * @param   Copy_u8AHPDiv: AHB prescaler value (AHB_NOT_DIV to AHB_DIV_512)
 * @return  RCC_ErrorState: Error status (RCC_OK, NOK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumAHBConfig(uint8_t Copy_u8AHPDiv)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    if (Copy_u8AHPDiv >= AHB_NOT_DIV || Copy_u8AHPDiv <= AHB_DIV_512)
    {
      /* Configure AHB prescaler in CFGR register */
      MRCC->CFGR &= (~(0xF << 4));
      MRCC->CFGR |= (Copy_u8AHPDiv << 4);
    }
    else
    {
      Local_u8ErrorState = NOK;
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Configure APB1 bus clock prescaler
 * @param   Copy_u8APB1Div: APB1 prescaler value (APB_NOT_DIV to APB_DIV_16)
 * @return  RCC_ErrorState: Error status (RCC_OK, NOK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumAPB1Config(uint8_t Copy_u8APB1Div)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    if (Copy_u8APB1Div >= APB_NOT_DIV || Copy_u8APB1Div <= APB_DIV_16)
    {
      /* Configure APB1 prescaler in CFGR register */
      MRCC->CFGR &= (~(0x7 << 10));
      MRCC->CFGR |= (Copy_u8APB1Div << 10);
    }
    else
    {
      Local_u8ErrorState = NOK;
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Configure APB2 bus clock prescaler
 * @param   Copy_u8APB2Div: APB2 prescaler value (APB_NOT_DIV to APB_DIV_16)
 * @return  RCC_ErrorState: Error status (RCC_OK, NOK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumAPB2Config(uint8_t Copy_u8APB2Div)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    if (Copy_u8APB2Div >= APB_NOT_DIV || Copy_u8APB2Div <= APB_DIV_16)
    {
      /* Configure APB2 prescaler in CFGR register */
      MRCC->CFGR &= (~(0x7 << 13));
      MRCC->CFGR |= (Copy_u8APB2Div << 13);
    }
    else
    {
      Local_u8ErrorState = NOK;
    }
    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Enable or disable peripheral clock on AHB buses
 * @param   Copy_u8Bus: AHB bus number (AHB1, AHB2, AHB3)
 * @param   Copy_u8AHPPer: Peripheral number on the selected bus
 * @param   Copy_u8Status: Peripheral clock status (PER_ON, PER_OFF)
 * @return  RCC_ErrorState: Error status (RCC_OK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumAHPPerSts(uint8_t Copy_u8Bus, uint8_t Copy_u8AHPPer, uint8_t Copy_u8Status)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;
  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;

    switch (Copy_u8Bus)
    {
    case AHB1:
      /* Configure AHB1 peripheral clock */
      if (Copy_u8Status == PER_ON)
      {
        SET_BIT(MRCC->AHP1ENR, Copy_u8AHPPer);
      }
      else if (Copy_u8Status == PER_OFF)
      {
        CLR_BIT(MRCC->AHP1ENR, Copy_u8AHPPer);
      }
      break;
    case AHB2:
      /* Configure AHB2 peripheral clock */
      if (Copy_u8Status == PER_ON)
      {
        SET_BIT(MRCC->AHP2ENR, Copy_u8AHPPer);
      }
      else if (Copy_u8Status == PER_OFF)
      {
        CLR_BIT(MRCC->AHP2ENR, Copy_u8AHPPer);
      }
      break;
    case AHB3:
      /* Configure AHB3 peripheral clock */
      if (Copy_u8Status == PER_ON)
      {
        SET_BIT(MRCC->AHP3ENR, Copy_u8AHPPer);
      }
      else if (Copy_u8Status == PER_OFF)
      {
        CLR_BIT(MRCC->AHP3ENR, Copy_u8AHPPer);
      }
      break;
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}

/**
 * @brief   Enable or disable peripheral clock on APB buses
 * @param   Copy_u8Bus: APB bus number (APB1, APB2)
 * @param   Copy_u8AHPPer: Peripheral number on the selected bus
 * @param   Copy_u8Status: Peripheral clock status (PER_ON, PER_OFF)
 * @return  RCC_ErrorState: Error status (RCC_OK, BUSY_STATE)
 */
RCC_ErrorState RCC_enumABPPerSts(uint8_t Copy_u8Bus, uint8_t Copy_u8AHPPer, uint8_t Copy_u8Status)
{
  RCC_ErrorState Local_u8ErrorState = RCC_OK;

  if (RCC_u8State == IDLE)
  {
    RCC_u8State = BUSY;
    switch (Copy_u8Bus)
    {
    case APB1:
      if (Copy_u8Status == PER_ON)
      {
        SET_BIT(MRCC->APB1ENR, Copy_u8AHPPer);
      }
      else if (Copy_u8Status == PER_OFF)
      {
        CLR_BIT(MRCC->APB1ENR, Copy_u8AHPPer);
      }
      else
      {
      }
      break;
    case APB2:
      if (Copy_u8Status == PER_ON)
      {
        SET_BIT(MRCC->APB2ENR, Copy_u8AHPPer);
      }
      else if (Copy_u8Status == PER_OFF)
      {
        CLR_BIT(MRCC->APB2ENR, Copy_u8AHPPer);
      }
      else
      {
      }
      break;

    default:
      break;
    }

    RCC_u8State = IDLE;
  }
  else
  {
    Local_u8ErrorState = BUSY_STATE;
  }

  return Local_u8ErrorState;
}
