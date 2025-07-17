/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    DMA_private.h     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : DMA                                             **
 **                                                                           **
 **===========================================================================**
 */

#ifndef MCAL_DMA_PRIVATE_H_
#define MCAL_DMA_PRIVATE_H_

/**
 * @defgroup DMA_Private_Defines DMA Private Defines
 * @brief Private defines for DMA configuration registers
 */

/**
 * @defgroup DMA_SxCR_Registers DMA Stream Control Register (SxCR) Defines
 * @brief Defines for configuring DMA stream control register
 */

/**
 * @brief DMA Stream Control Register masks and bit positions
 */
#define MDMA_SxCR_EN_MASK        (0X1 << 0)    /**< Stream Enable bit mask */
#define MDMA_SxCR_EN             0            /**< Stream Enable bit position */
#define MDMA_SxCR_DMEIE_MASK     (0X1 << 1)    /**< Direct Mode Error Interrupt Enable bit mask */
#define MDMA_SxCR_DMEIE          1            /**< Direct Mode Error Interrupt Enable bit position */
#define MDMA_SxCR_TEIE_MASK      (0X1 << 2)    /**< Transfer Error Interrupt Enable bit mask */
#define MDMA_SxCR_TEIE           2            /**< Transfer Error Interrupt Enable bit position */
#define MDMA_SxCR_HTIE_MASK      (0X1 << 3)    /**< Half Transfer Interrupt Enable bit mask */
#define MDMA_SxCR_HTIE           3            /**< Half Transfer Interrupt Enable bit position */
#define MDMA_SxCR_TCIE_MASK      (0X1 << 4)    /**< Transfer Complete Interrupt Enable bit mask */
#define MDMA_SxCR_TCIE           4            /**< Transfer Complete Interrupt Enable bit position */
#define MDMA_SxCR_PFCTR_MASK     (0X1 << 5)    /**< Peripheral Flow Controller bit mask */
#define MDMA_SxCR_PFCTR          5            /**< Peripheral Flow Controller bit position */
#define MDMA_SxCR_DIR_MASK       (0X3 << 6)    /**< Data Transfer Direction bit mask */
#define MDMA_SxCR_DIR            6            /**< Data Transfer Direction bit position */
#define MDMA_SxCR_CIRC_MASK      (0X1 << 8)    /**< Circular Mode bit mask */
#define MDMA_SxCR_CIRC           8            /**< Circular Mode bit position */
#define MDMA_SxCR_PINC_MASK      (0X1 << 9)    /**< Peripheral Increment Mode bit mask */
#define MDMA_SxCR_PINC           9            /**< Peripheral Increment Mode bit position */
#define MDMA_SxCR_MINC_MASK      (0X1 << 10)   /**< Memory Increment Mode bit mask */
#define MDMA_SxCR_MINC           10           /**< Memory Increment Mode bit position */
#define MDMA_SxCR_PSIZE_MASK     (0X3 << 11)   /**< Peripheral Size bit mask */
#define MDMA_SxCR_PSIZE          11           /**< Peripheral Size bit position */
#define MDMA_SxCR_MSIZE_MASK     (0X3 << 13)   /**< Memory Size bit mask */
#define MDMA_SxCR_MSIZE          13           /**< Memory Size bit position */
#define MDMA_SxCR_PL_MASK        (0X3 << 16)   /**< Channel Priority Level bit mask */
#define MDMA_SxCR_PL             16           /**< Channel Priority Level bit position */
#define MDMA_SxCR_DBM_MASK       (0X1 << 18)   /**< Double Buffer Mode bit mask */
#define MDMA_SxCR_DBM            18           /**< Double Buffer Mode bit position */
#define MDMA_SxCR_MBURST_MASK    (0X3 << 21)   /**< Memory Burst Transfer Configuration bit mask */
#define MDMA_SxCR_MBURST         21           /**< Memory Burst Transfer Configuration bit position */
#define MDMA_SxCR_PBURST_MASK    (0X3 << 23)   /**< Peripheral Burst Transfer Configuration bit mask */
#define MDMA_SxCR_PBURST         23           /**< Peripheral Burst Transfer Configuration bit position */
#define MDMA_SxCR_CHSEL_MASK     (0X7 << 25)   /**< Channel Selection bit mask */
#define MDMA_SxCR_CHSEL          25           /**< Channel Selection bit position */

/**
 * @defgroup DMA_SxFCR_Registers DMA Stream FIFO Control Register (SxFCR) Defines
 * @brief Defines for configuring DMA stream FIFO control register
 */

/**
 * @brief DMA Stream FIFO Control Register masks and bit positions
 */
#define MDMA_SxFCR_DMDIS_MASK    (0X1 << 2)    /**< Direct Mode Disable bit mask */
#define MDMA_SxFCR_DMDIS         2            /**< Direct Mode Disable bit position */
#define MDMA_SxFCR_FTH_MASK      (0X3 << 0)    /**< FIFO Threshold Selection bit mask */
#define MDMA_SxFCR_FTH           0            /**< FIFO Threshold Selection bit position */

/**
 * @brief Maximum number of DMA streams
 */
#define MAXIMUM_NUM_OF_STREAMS   8

/**
 * @brief DMA interrupt flag check function for DMA1
 * @param Copy_u8Stream: Stream number (0-7)
 * @param Copy_u8Flag: Flag to check
 * @return uint8_t: 1 if flag is set, 0 if flag is not set
 */
static uint8_t DMA1_u8CheckIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag);

/**
 * @brief DMA interrupt flag clear function for DMA1
 * @param Copy_u8Stream: Stream number (0-7)
 * @param Copy_u8Flag: Flag to clear
 */
static void DMA1_vClearIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag);

/**
 * @brief DMA interrupt flag check function for DMA2
 * @param Copy_u8Stream: Stream number (0-7)
 * @param Copy_u8Flag: Flag to check
 * @return uint8_t: 1 if flag is set, 0 if flag is not set
 */
static uint8_t DMA2_u8CheckIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag);

/**
 * @brief DMA interrupt flag clear function for DMA2
 * @param Copy_u8Stream: Stream number (0-7)
 * @param Copy_u8Flag: Flag to clear
 */
static void DMA2_vClearIF(uint8_t Copy_u8Stream, uint8_t Copy_u8Flag);

#endif /* MCAL_DMA_PRIVATE_H_ */
