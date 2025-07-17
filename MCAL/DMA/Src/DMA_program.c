/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    DMA_program.c     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : DMA                                             **
 **                                                                           **
 **===========================================================================**
 */

#include <stdint.h>

#include "STM32F446xx.h"
#include "ErrTypes.h"

#include "DMA_interface.h"
#include "DMA_private.h"
#include "DMA_config.h"

/**
 * @brief DMA1 Callback function arrays for different interrupt types
 * Each array holds callback functions for DMA1 streams (0-7)
 */
void (*DMA1_DME_CallBack[MAXIMUM_NUM_OF_STREAMS]) (void) = {NULL};  /**< Direct Mode Error Callbacks */
void (*DMA1_TE_CallBack[MAXIMUM_NUM_OF_STREAMS])  (void) = {NULL};  /**< Transfer Error Callbacks */
void (*DMA1_HT_CallBack[MAXIMUM_NUM_OF_STREAMS])  (void) = {NULL};  /**< Half Transfer Callbacks */
void (*DMA1_TC_CallBack[MAXIMUM_NUM_OF_STREAMS])  (void) = {NULL};  /**< Transfer Complete Callbacks */

/**
 * @brief DMA2 Callback function arrays for different interrupt types
 * Each array holds callback functions for DMA2 streams (0-7)
 */
void (*DMA2_DME_CallBack[MAXIMUM_NUM_OF_STREAMS]) (void) = {NULL};  /**< Direct Mode Error Callbacks */
void (*DMA2_TE_CallBack[MAXIMUM_NUM_OF_STREAMS])  (void) = {NULL};  /**< Transfer Error Callbacks */
void (*DMA2_HT_CallBack[MAXIMUM_NUM_OF_STREAMS])  (void) = {NULL};  /**< Half Transfer Callbacks */
void (*DMA2_TC_CallBack[MAXIMUM_NUM_OF_STREAMS])  (void) = {NULL};  /**< Transfer Complete Callbacks */

/*=================================================================================================================*/
/**
 * @fn     DMA1_enumSetConfig
 * @brief  Configures a DMA1 stream with the provided settings
 * @param  Copy_pstConfig: Pointer to the DMA configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration completed successfully
 *         - NULL_POINTER: Invalid configuration pointer
 *
 * @details This function configures various DMA stream parameters including:
 *          - Stream enable/disable
 *          - Channel selection
 *          - Priority level
 *          - Transfer direction (Memory-to-Memory, Memory-to-Peripheral, Peripheral-to-Memory)
 *          - Peripheral and Memory increment modes
 *          - Data size for peripheral and memory transfers
 *          - Circular mode
 *          - FIFO configuration (if enabled)
 */
ErrorState_t DMA1_enumSetConfig(DMA_Config_t *Copy_pstConfig)
{
  uint8_t Local_u8ErrorState = OK;

  if(Copy_pstConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    // Disable stream before configuration
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_EN_MASK;

    // Configure DMA stream parameters
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CHSEL_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Channel << MDMA_SxCR_CHSEL;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PL_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Priority << MDMA_SxCR_PL;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_DIR_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Direction << MDMA_SxCR_DIR;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PINC_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeripheralInc << MDMA_SxCR_PINC;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MINC_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemoryInc << MDMA_SxCR_MINC;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PSIZE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeriphDataAlignment << MDMA_SxCR_PSIZE;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MSIZE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemDataAlignment << MDMA_SxCR_MSIZE;

    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CIRC_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->CircularMode << MDMA_SxCR_CIRC;

    // Configure FIFO settings if FIFO mode is enabled and not in Memory-to-Memory mode
    if (Copy_pstConfig->FIFOMode == DMA_FIFO_MODE && Copy_pstConfig->Direction != DMA_M2M)
    {
      MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PBURST_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PBURST << MDMA_SxCR_PBURST;

      MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MBURST_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MBURST << MDMA_SxCR_MBURST;

      MDMA1->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_FTH_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOTHRESHOLD_LEVEL << MDMA_SxFCR_FTH;

      /* Choose FIFO Mode */
      MDMA1->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_DMDIS_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
    else
    {
      MDMA1->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
  }
  return Local_u8ErrorState;
}


/*=================================================================================================================*/
/**
 * @fn     DMA2_enumSetConfig
 * @brief  This function configures the DMA stream
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration done successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA2_enumSetConfig(DMA_Config_t *Copy_pstConfig)
{
  uint8_t Local_u8ErrorState = OK;

  if(Copy_pstConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    /* Clear the stream enable bit */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_EN_MASK;

    /*Choose Channel */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CHSEL_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Channel << MDMA_SxCR_CHSEL;

    /*Choose Priority */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PL_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Priority << MDMA_SxCR_PL;

    /*Choose Direction */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_DIR_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Direction << MDMA_SxCR_DIR;

    /*Choose Peripheral Increment */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PINC_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeripheralInc << MDMA_SxCR_PINC;

    /*Choose Memory Increment */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MINC_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemoryInc << MDMA_SxCR_MINC;

    /*Choose Peripheral Size */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PSIZE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeriphDataAlignment << MDMA_SxCR_PSIZE;

    /*Choose Memory Size */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MSIZE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemDataAlignment << MDMA_SxCR_MSIZE;

    /*Choose Circular Mode */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CIRC_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->CircularMode << MDMA_SxCR_CIRC;

    if (Copy_pstConfig->FIFOMode == DMA_FIFO_MODE && Copy_pstConfig->Direction != DMA_M2M)
    {
      /* Choose PBURST */
      MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PBURST_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PBURST << MDMA_SxCR_PBURST;

      /* Choose MBURST */
      MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MBURST_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MBURST << MDMA_SxCR_MBURST;

      /* Choose FIFO Threshold */
      MDMA2->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_FTH_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOTHRESHOLD_LEVEL << MDMA_SxFCR_FTH;

      /* Choose FIFO Mode */
      MDMA2->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_DMDIS_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
    else
    {
      MDMA2->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
  }
  return Local_u8ErrorState;
}
/*=================================================================================================================*/
/**
 * @fn     DMA1_enumSetConfigIT
 * @brief  Configures DMA1 stream with interrupt handling
 * @param  Copy_pstConfig: Pointer to DMA handle structure
 * @return ErrorState_t:
 *         - OK: Configuration completed successfully
 *         - NULL_POINTER: Invalid configuration pointer
 * 
 * @details This function configures DMA1 stream with interrupt handling capabilities:
 *          - Configures basic DMA parameters (channel, priority, direction, etc.)
 *          - Sets up interrupt enable bits (DMEIE, TEIE, HTIE, TCIE)
 *          - Registers callback functions for different interrupt types
 *          - Configures FIFO settings if FIFO mode is enabled
 *          - Validates callback functions before registration
 */

ErrorState_t DMA1_enumSetConfigIT(DMA_Hundle_t *Copy_pstConfig)
{
  uint8_t Local_u8ErrorState = OK;

  if(Copy_pstConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    // Disable stream before configuration
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_EN_MASK;

    // Configure DMA stream parameters
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CHSEL_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Channel << MDMA_SxCR_CHSEL;

    /*Choose Priority */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PL_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Priority << MDMA_SxCR_PL;

    /*Choose Direction */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_DIR_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Direction << MDMA_SxCR_DIR;

    /*Choose Peripheral Increment */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PINC_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeripheralInc << MDMA_SxCR_PINC;

    /*Choose Memory Increment */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MINC_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemoryInc << MDMA_SxCR_MINC;

    /*Choose Peripheral Size */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PSIZE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeriphDataAlignment << MDMA_SxCR_PSIZE;

    /*Choose Memory Size */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MSIZE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemDataAlignment << MDMA_SxCR_MSIZE;

    /*Choose Circular Mode */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CIRC_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->CircularMode << MDMA_SxCR_CIRC;

    // Configure interrupt enables
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_DMEIE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->DMEIE << MDMA_SxCR_DMEIE;

    /* Transfer Error Interrupt Enable */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_TEIE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->TEIE << MDMA_SxCR_TEIE;

    /* Half Transfer Interrupt Enable */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_HTIE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->HTIE << MDMA_SxCR_HTIE;

    /* Transfer Complete Interrupt Enable */
    MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_TCIE_MASK;
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->TCIE << MDMA_SxCR_TCIE;

    // Register callback functions for enabled interrupts
    if (Copy_pstConfig->DMEIE == DMA_DMEIE_EN)
    {
      if (Copy_pstConfig->pfnDMECallback != NULL)
      {
        DMA1_DME_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnDMECallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA1_DME_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->TEIE == DMA_TEIE_EN)
    {
      if (Copy_pstConfig->pfnTECallback != NULL)
      {
        DMA1_TE_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnTECallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA1_TE_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->HTIE == DMA_HTIE_EN)
    {
      if (Copy_pstConfig->pfnHTCallback != NULL)
      {
        DMA1_HT_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnHTCallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA1_HT_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->TCIE == DMA_TCIE_EN)
    {
      if (Copy_pstConfig->pfnTCCallback != NULL)
      {
        DMA1_TC_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnTCCallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA1_TC_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    // Configure FIFO settings if FIFO mode is enabled
    if (Copy_pstConfig->FIFOMode == DMA_FIFO_MODE && Copy_pstConfig->Direction != DMA_M2M)
    {
      /* Choose PBURST */
      MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PBURST_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PBURST << MDMA_SxCR_PBURST;

      /* Choose MBURST */
      MDMA1->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MBURST_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MBURST << MDMA_SxCR_MBURST;

      /* Choose FIFO Threshold */
      MDMA1->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_FTH_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOTHRESHOLD_LEVEL << MDMA_SxFCR_FTH;

      /* Choose FIFO Mode */
      MDMA1->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_DMDIS_MASK;
      MDMA1->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
    else
    {
      MDMA1->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_enumSetConfigIT
 * @brief  This function configures the DMA stream interrupt
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration done successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA2_enumSetConfigIT(DMA_Hundle_t *Copy_pstConfig)
{
  uint8_t Local_u8ErrorState = OK;

  if(Copy_pstConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    /* Clear the stream enable bit */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_EN_MASK;

    /*Choose Channel */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CHSEL_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Channel << MDMA_SxCR_CHSEL;

    /*Choose Priority */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PL_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Priority << MDMA_SxCR_PL;

    /*Choose Direction */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_DIR_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->Direction << MDMA_SxCR_DIR;

    /*Choose Peripheral Increment */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PINC_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeripheralInc << MDMA_SxCR_PINC;

    /*Choose Memory Increment */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MINC_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemoryInc << MDMA_SxCR_MINC;

    /*Choose Peripheral Size */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PSIZE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PeriphDataAlignment << MDMA_SxCR_PSIZE;

    /*Choose Memory Size */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MSIZE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MemDataAlignment << MDMA_SxCR_MSIZE;

    /*Choose Circular Mode */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_CIRC_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->CircularMode << MDMA_SxCR_CIRC;

    /* Direct Mode Interrupt */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_DMEIE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->DMEIE << MDMA_SxCR_DMEIE;

    /* Transfer Error Interrupt Enable */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_TEIE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->TEIE << MDMA_SxCR_TEIE;

    /* Half Transfer Interrupt Enable */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_HTIE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->HTIE << MDMA_SxCR_HTIE;

    /* Transfer Complete Interrupt Enable */
    MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_TCIE_MASK;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->TCIE << MDMA_SxCR_TCIE;

    if (Copy_pstConfig->DMEIE == DMA_DMEIE_EN)
    {
      if (Copy_pstConfig->pfnDMECallback != NULL)
      {
        DMA2_DME_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnDMECallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA2_DME_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->TEIE == DMA_TEIE_EN)
    {
      if (Copy_pstConfig->pfnTECallback != NULL)
      {
        DMA2_TE_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnTECallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA2_TE_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->HTIE == DMA_HTIE_EN)
    {
      if (Copy_pstConfig->pfnHTCallback != NULL)
      {
        DMA2_HT_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnHTCallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA2_HT_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->TCIE == DMA_TCIE_EN)
    {
      if (Copy_pstConfig->pfnTCCallback != NULL)
      {
        DMA2_TC_CallBack[Copy_pstConfig->Stream] = Copy_pstConfig->pfnTCCallback;
      }
      else
      {
        return NULL_POINTER;
      }
    }
    else
    {
      DMA2_TC_CallBack[Copy_pstConfig->Stream] = NULL;
    }

    if (Copy_pstConfig->FIFOMode == DMA_FIFO_MODE && Copy_pstConfig->Direction != DMA_M2M)
    {
      /* Choose PBURST */
      MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_PBURST_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->PBURST << MDMA_SxCR_PBURST;

      /* Choose MBURST */
      MDMA2->Stream[Copy_pstConfig->Stream].CR &= ~MDMA_SxCR_MBURST_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].CR |= Copy_pstConfig->MBURST << MDMA_SxCR_MBURST;

      /* Choose FIFO Threshold */
      MDMA2->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_FTH_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOTHRESHOLD_LEVEL << MDMA_SxFCR_FTH;

      /* Choose FIFO Mode */
      MDMA2->Stream[Copy_pstConfig->Stream].FCR &= ~MDMA_SxFCR_DMDIS_MASK;
      MDMA2->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
    else
    {
      MDMA2->Stream[Copy_pstConfig->Stream].FCR |= Copy_pstConfig->FIFOMode << MDMA_SxFCR_DMDIS;
    }
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_enumStartTransfer
 * @brief  Initiate DMA1 Transfer
 * 
 * @param  Copy_pstConfig: Pointer to DMA configuration structure
 * @param  Copy_u32SrcAddress: Source memory address
 * @param  Copy_u32DstAddress: Destination memory address
 * @param  Copy_u32NumOfData: Number of data items to transfer
 * @return ErrorState_t:
 *         - OK: Transfer started successfully
 *         - NULL_POINTER: Invalid configuration pointer
 * 
 * @details This function initiates a DMA transfer by:
 *          1. Configuring source and destination addresses
 *          2. Setting the transfer count
 *          3. Enabling the DMA stream
 *          
 * @note   The DMA stream must be configured before calling this function
 */
ErrorState_t DMA1_enumStartTransfer(DMA_Config_t *Copy_pstConfig, uint32_t* Copy_u32SrcAddress, uint32_t* Copy_u32DstAddress, uint32_t Copy_u32NumOfData)
{
  uint8_t Local_u8ErrorState = OK;

  if(Copy_pstConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    // Set source address in peripheral address register
    MDMA1->Stream[Copy_pstConfig->Stream].PAR = (uint32_t)Copy_u32SrcAddress;

    // Set destination address in memory address register
    MDMA1->Stream[Copy_pstConfig->Stream].M0AR = (uint32_t)Copy_u32DstAddress;

    // Set number of data items to transfer in number of data to transfer register
    MDMA1->Stream[Copy_pstConfig->Stream].NDTR = Copy_u32NumOfData;

    // Enable DMA stream by setting the EN bit
    MDMA1->Stream[Copy_pstConfig->Stream].CR |= (1 << MDMA_SxCR_EN);
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_enumStartTransfer
 * @brief  This function starts the DMA transfer
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @param  Copy_u32SrcAddress: Source address
 * @param  Copy_u32DstAddress: Destination address
 * @param  Copy_u32NumOfData: Number of data to transfer
 * @return ErrorState_t:
 *         - OK: Transfer started successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA2_enumStartTransfer(DMA_Config_t *Copy_pstConfig, uint32_t* Copy_u32SrcAddress, uint32_t* Copy_u32DstAddress, uint32_t Copy_u32NumOfData)
{
  uint8_t Local_u8ErrorState = OK;

  if(Copy_pstConfig == NULL)
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  else
  {
    if (Copy_pstConfig->Direction == DMA_M2P)
    {
      MDMA2->Stream[Copy_pstConfig->Stream].PAR = (uint32_t)Copy_u32DstAddress;
      MDMA2->Stream[Copy_pstConfig->Stream].M0AR = (uint32_t)Copy_u32SrcAddress;
    }
    else
    {
      MDMA2->Stream[Copy_pstConfig->Stream].PAR = (uint32_t)Copy_u32SrcAddress;
      MDMA2->Stream[Copy_pstConfig->Stream].M0AR = (uint32_t)Copy_u32DstAddress;
    }
    MDMA2->Stream[Copy_pstConfig->Stream].NDTR = Copy_u32NumOfData;
    MDMA2->Stream[Copy_pstConfig->Stream].CR |= 1 << MDMA_SxCR_EN;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_u8CheckIF
 * @brief  This function checks the DMA stream interrupt flag
 * @param  Copy_u8Stream: Stream number
 * @param  Copy_u8Flag: Flag to check
 * @return uint8_t:
 *         - 1: Flag is set
 *         - 0: Flag is not set
 */
static uint8_t DMA1_u8CheckIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag)
{
  uint8_t Local_u8Temp = 0;
  if (Copy_u8Stream < 2)
  {
    Local_u8Temp = Copy_u8Stream * 6 + Copy_u8Flag;
    return (MDMA1->LISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else if (Copy_u8Stream < 4)
  {
    Local_u8Temp = (((Copy_u8Stream - 2) * 6 + Copy_u8Flag) + 16);
    return (MDMA1->LISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else if (Copy_u8Stream < 6)
  {
    Local_u8Temp = ((Copy_u8Stream - 4) * 6 + Copy_u8Flag);
    return (MDMA1->HISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else if (Copy_u8Stream < 8)
  {
    Local_u8Temp = (((Copy_u8Stream - 6) * 6 + Copy_u8Flag) + 16);
    return (MDMA1->HISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else
  {
    return 0;
  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_u8CheckIF
 * @brief  This function checks the DMA stream interrupt flag
 * @param  Copy_u8Stream: Stream number
 * @param  Copy_u8Flag: Flag to check
 * @return uint8_t:
 *         - 1: Flag is set
 *         - 0: Flag is not set
 */
static uint8_t DMA2_u8CheckIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag)
{
  uint8_t Local_u8Temp = 0;
  if (Copy_u8Stream < 2)
  {
    Local_u8Temp = Copy_u8Stream * 6 + Copy_u8Flag;
    return (MDMA2->LISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else if (Copy_u8Stream < 4)
  {
    Local_u8Temp = (((Copy_u8Stream - 2) * 6 + Copy_u8Flag) + 16);
    return (MDMA2->LISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else if (Copy_u8Stream < 6)
  {
    Local_u8Temp = ((Copy_u8Stream - 4) * 6 + Copy_u8Flag);
    return (MDMA2->HISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else if (Copy_u8Stream < 8)
  {
    Local_u8Temp = (((Copy_u8Stream - 6) * 6 + Copy_u8Flag) + 16);
    return (MDMA2->HISR & (1 << Local_u8Temp)) >> Local_u8Temp;
  }
  else
  {
    return 0;
  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_vClearIF
 * @brief  This function clears the DMA stream interrupt flag
 * @param  Copy_u8Stream: Stream number
 * @param  Copy_u8Flag: Flag to clear
 */
static void DMA1_vClearIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag)
{
  uint8_t Local_u8Temp = 0;
  if (Copy_u8Stream < 2)
  {
    Local_u8Temp = Copy_u8Stream * 6 + Copy_u8Flag;
    MDMA1->LIFCR |= (1 << Local_u8Temp);
  }
  else if (Copy_u8Stream < 4)
  {
    Local_u8Temp = (((Copy_u8Stream - 2) * 6 + Copy_u8Flag) + 16);
    MDMA1->LIFCR |= (1 << Local_u8Temp);
  }
  else if (Copy_u8Stream < 6)
  {
    Local_u8Temp = ((Copy_u8Stream - 4) * 6 + Copy_u8Flag);
    MDMA1->HIFCR |= (1 << Local_u8Temp);
  }
  else if (Copy_u8Stream < 8)
  {
    Local_u8Temp = (((Copy_u8Stream - 6) * 6 + Copy_u8Flag) + 16);
    MDMA1->HIFCR |= (1 << Local_u8Temp);
  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_vClearIF
 * @brief  This function clears the DMA stream interrupt flag
 * @param  Copy_u8Stream: Stream number
 * @param  Copy_u8Flag: Flag to clear
 */
static void DMA2_vClearIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag)
{
  uint8_t Local_u8Temp = 0;
  if (Copy_u8Stream < 2)
  {
    Local_u8Temp = Copy_u8Stream * 6 + Copy_u8Flag;
    MDMA2->LIFCR |= (1 << Local_u8Temp);
  }
  else if (Copy_u8Stream < 4)
  {
    Local_u8Temp = (((Copy_u8Stream - 2) * 6 + Copy_u8Flag) + 16);
    MDMA2->LIFCR |= (1 << Local_u8Temp);
  }
  else if (Copy_u8Stream < 6)
  {
    Local_u8Temp = ((Copy_u8Stream - 4) * 6 + Copy_u8Flag);
    MDMA2->HIFCR |= (1 << Local_u8Temp);
  }
  else if (Copy_u8Stream < 8)
  {
    Local_u8Temp = (((Copy_u8Stream - 6) * 6 + Copy_u8Flag) + 16);
    MDMA2->HIFCR |= (1 << Local_u8Temp);
  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream0_IRQHandler
 * @brief  DMA1 Stream 0 Interrupt Handler
 * 
 * @details This interrupt handler manages all interrupt sources for DMA1 Stream 0:
 *          - Direct Mode Error Interrupt (DMEIE)
 *          - Transfer Error Interrupt (TEIE)
 *          - Half Transfer Interrupt (HTIE)
 *          - Transfer Complete Interrupt (TCIE)
 *          
 * @note   Interrupt flags are cleared before calling the corresponding callback
 */
void DMA1_Stream0_IRQHandler(void)
{
  // Check and handle Direct Mode Error Interrupt
  if (DMA1_u8CheckIF(DMA_STREAM_0, DMA_DMEIF) == 1)
  {
    DMA1_vClearIF(DMA_STREAM_0, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_0]();
    }
  }
  // Check and handle Transfer Error Interrupt
  else if (DMA1_u8CheckIF(DMA_STREAM_0, DMA_TEIF) == 1)
  {
    DMA1_vClearIF(DMA_STREAM_0, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_0]();
    }
  }
  // Check and handle Half Transfer Interrupt
  else if (DMA1_u8CheckIF(DMA_STREAM_0, DMA_HTIF) == 1)
  {
    DMA1_vClearIF(DMA_STREAM_0, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_0]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream1_IRQHandler
 * @brief  This function handles the DMA stream 1 interrupt
 */
void DMA1_Stream1_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_1, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_1, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_1]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_1, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_1, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_1]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_1, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_1, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_1]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_1, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_1, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_1]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream2_IRQHandler
 * @brief  This function handles the DMA stream 2 interrupt
 */
void DMA1_Stream2_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_2, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_2, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_2]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_2, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_2, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_2]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_2, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_2, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_2]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_2, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_2, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_2]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream3_IRQHandler
 * @brief  This function handles the DMA stream 3 interrupt
 */
void DMA1_Stream3_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_3, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_3, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_3]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_3, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_3, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_3]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_3, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_3, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_3]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_3, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_3, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_3]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream4_IRQHandler
 * @brief  This function handles the DMA stream 4 interrupt
 */
void DMA1_Stream4_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_4, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_4, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_4]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_4, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_4, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_4]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_4, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_4, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_4]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_4, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_4, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_4]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream5_IRQHandler
 * @brief  This function handles the DMA stream 5 interrupt
 */
void DMA1_Stream5_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_5, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_5, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_5]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_5, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_5, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_5]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_5, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_5, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_5]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_5, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_5, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_5]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream6_IRQHandler
 * @brief  This function handles the DMA stream 6 interrupt
 */
void DMA1_Stream6_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_6, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_6, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_6]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_6, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_6, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_6]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_6, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_6, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_6]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_6, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_6, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_6]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA1_Stream7_IRQHandler
 * @brief  This function handles the DMA stream 7 interrupt
 */
void DMA1_Stream7_IRQHandler(void)
{
  if (DMA1_u8CheckIF(DMA_STREAM_7, DMA_DMEIF))
  {
    DMA1_vClearIF(DMA_STREAM_7, DMA_DMEIF);
    if (DMA1_DME_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA1_DME_CallBack[DMA_STREAM_7]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_7, DMA_TEIF))
  {
    DMA1_vClearIF(DMA_STREAM_7, DMA_TEIF);
    if (DMA1_TE_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA1_TE_CallBack[DMA_STREAM_7]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_7, DMA_TCIF))
  {
    DMA1_vClearIF(DMA_STREAM_7, DMA_TCIF);
    if (DMA1_TC_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA1_TC_CallBack[DMA_STREAM_7]();
    }
  }
  else if (DMA1_u8CheckIF(DMA_STREAM_7, DMA_HTIF))
  {
    DMA1_vClearIF(DMA_STREAM_7, DMA_HTIF);
    if (DMA1_HT_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA1_HT_CallBack[DMA_STREAM_7]();
    }
  }
  else
  {

  }
}



/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream0_IRQHandler
 * @brief  This function handles the DMA stream 0 interrupt
 */
void DMA2_Stream0_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_0, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_0, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_0]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_0, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_0, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_0]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_0, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_0, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_0]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_0, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_0, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_0] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_0]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream1_IRQHandler
 * @brief  This function handles the DMA stream 1 interrupt
 */
void DMA2_Stream1_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_1, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_1, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_1]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_1, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_1, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_1]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_1, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_1, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_1]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_1, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_1, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_1] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_1]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream2_IRQHandler
 * @brief  This function handles the DMA stream 2 interrupt
 */
void DMA2_Stream2_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_2, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_2, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_2]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_2, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_2, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_2]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_2, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_2, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_2]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_2, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_2, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_2] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_2]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream3_IRQHandler
 * @brief  This function handles the DMA stream 3 interrupt
 */
void DMA2_Stream3_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_3, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_3, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_3]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_3, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_3, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_3]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_3, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_3, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_3]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_3, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_3, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_3] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_3]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream4_IRQHandler
 * @brief  This function handles the DMA stream 4 interrupt
 */
void DMA2_Stream4_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_4, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_4, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_4]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_4, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_4, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_4]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_4, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_4, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_4]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_4, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_4, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_4] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_4]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream5_IRQHandler
 * @brief  This function handles the DMA stream 5 interrupt
 */
void DMA2_Stream5_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_5, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_5, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_5]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_5, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_5, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_5]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_5, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_5, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_5]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_5, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_5, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_5] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_5]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream6_IRQHandler
 * @brief  This function handles the DMA stream 6 interrupt
 */
void DMA2_Stream6_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_6, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_6, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_6]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_6, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_6, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_6]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_6, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_6, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_6]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_6, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_6, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_6] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_6]();
    }
  }
  else
  {

  }
}

/*=================================================================================================================*/
/**
 * @fn     DMA2_Stream7_IRQHandler
 * @brief  This function handles the DMA stream 7 interrupt
 */
void DMA2_Stream7_IRQHandler(void)
{
  if (DMA2_u8CheckIF(DMA_STREAM_7, DMA_DMEIF))
  {
    DMA2_vClearIF(DMA_STREAM_7, DMA_DMEIF);
    if (DMA2_DME_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA2_DME_CallBack[DMA_STREAM_7]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_7, DMA_TEIF))
  {
    DMA2_vClearIF(DMA_STREAM_7, DMA_TEIF);
    if (DMA2_TE_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA2_TE_CallBack[DMA_STREAM_7]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_7, DMA_TCIF))
  {
    DMA2_vClearIF(DMA_STREAM_7, DMA_TCIF);
    if (DMA2_TC_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA2_TC_CallBack[DMA_STREAM_7]();
    }
  }
  else if (DMA2_u8CheckIF(DMA_STREAM_7, DMA_HTIF))
  {
    DMA2_vClearIF(DMA_STREAM_7, DMA_HTIF);
    if (DMA2_HT_CallBack[DMA_STREAM_7] != NULL)
    {
      DMA2_HT_CallBack[DMA_STREAM_7]();
    }
  }
  else
  {

  }
}
