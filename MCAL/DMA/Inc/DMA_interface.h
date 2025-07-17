/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    DMA_interface.h     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : DMA                                             **
 **                                                                           **
 **===========================================================================**
 */

#ifndef DMA_INTERFACE_H
#define DMA_INTERFACE_H

/**
 * @defgroup DMA_Configuration_Enums DMA Configuration Enums
 * @brief Configuration enums for DMA controller
 */

/**
 * @brief DMA Stream numbers (0-7)
 */
typedef enum
{
  DMA_STREAM_0,  /**< Stream 0 */
  DMA_STREAM_1,  /**< Stream 1 */
  DMA_STREAM_2,  /**< Stream 2 */
  DMA_STREAM_3,  /**< Stream 3 */
  DMA_STREAM_4,  /**< Stream 4 */
  DMA_STREAM_5,  /**< Stream 5 */
  DMA_STREAM_6,  /**< Stream 6 */
  DMA_STREAM_7   /**< Stream 7 */
} DMA_Stream_t;

/**
 * @brief DMA Channel numbers (0-7)
 */
typedef enum
{
  DMA_CHANNEL_0,  /**< Channel 0 */
  DMA_CHANNEL_1,  /**< Channel 1 */
  DMA_CHANNEL_2,  /**< Channel 2 */
  DMA_CHANNEL_3,  /**< Channel 3 */
  DMA_CHANNEL_4,  /**< Channel 4 */
  DMA_CHANNEL_5,  /**< Channel 5 */
  DMA_CHANNEL_6,  /**< Channel 6 */
  DMA_CHANNEL_7   /**< Channel 7 */
} DMA_Channel_t;

/**
 * @brief DMA Priority levels
 */
typedef enum
{
  DMA_LOW_PRIORITY,         /**< Lowest priority */
  DMA_MEDIUM_PRIORITY,      /**< Medium priority */
  DMA_HIGH_PRIORITY,        /**< High priority */
  DMA_VERY_HIGH_PRIORITY    /**< Very high priority */
} DMA_Priority_t;

/**
 * @brief DMA Transfer directions
 */
typedef enum
{
  DMA_P2M,  /**< Peripheral to Memory */
  DMA_M2P,  /**< Memory to Peripheral */
  DMA_M2M   /**< Memory to Memory */
} DMA_Direction_t;

/**
 * @brief DMA Double Buffer Mode
 */
typedef enum
{
  DMA_DBM_DIS,  /**< Double Buffer Mode disabled */
  DMA_DBM_EN   /**< Double Buffer Mode enabled */
} DMA_DoubleBufferMode_t;

/**
 * @brief DMA Memory Burst Transfer Configuration
 */
typedef enum
{
  DMA_MBURST_SINGLE_TRANSFER,  /**< Single transfer */
  DMA_MBURST_INCR4_TRANSFER,   /**< Increment by 4 */
  DMA_MBURST_INCR8_TRANSFER,   /**< Increment by 8 */
  DMA_MBURST_INCR16_TRANSFER   /**< Increment by 16 */
} DMA_MBURST_t;

/**
 * @brief DMA Peripheral Burst Transfer Configuration
 */
typedef enum
{
  DMA_PBURST_SINGLE_TRANSFER,  /**< Single transfer */
  DMA_PBURST_INCR4_TRANSFER,   /**< Increment by 4 */
  DMA_PBURST_INCR8_TRANSFER,   /**< Increment by 8 */
  DMA_PBURST_INCR16_TRANSFER   /**< Increment by 16 */
} DMA_PBURST_t;

/**
 * @brief DMA Memory Size Configuration
 */
typedef enum
{
  DMA_MEMORY_SIZE_BYTE,       /**< Byte (8-bit) */
  DMA_MEMORY_SIZE_HALFWORD,   /**< Halfword (16-bit) */
  DMA_MEMORY_SIZE_WORD        /**< Word (32-bit) */
} DMA_MemorySize_t;

/**
 * @brief DMA Peripheral Size Configuration
 */
typedef enum
{
  DMA_PERIPHERAL_SIZE_BYTE,       /**< Byte (8-bit) */
  DMA_PERIPHERAL_SIZE_HALFWORD,   /**< Halfword (16-bit) */
  DMA_PERIPHERAL_SIZE_WORD        /**< Word (32-bit) */
} DMA_PeripheralSize_t;

/**
 * @brief DMA Increment Mode for Peripheral
 */
typedef enum
{
  DMA_PINC_DIS,  /**< Disable peripheral increment */
  DMA_PINC_EN   /**< Enable peripheral increment */
} DMA_PeripheralInc_t;

/**
 * @brief DMA Increment Mode for Memory
 */
typedef enum
{
  DMA_MINC_DIS,  /**< Disable memory increment */
  DMA_MINC_EN   /**< Enable memory increment */
} DMA_MemoryInc_t;

/**
 * @brief DMA Circular Mode
 */
typedef enum
{
  DMA_CIRC_DIS,  /**< Disable circular mode */
  DMA_CIRC_EN   /**< Enable circular mode */
} DMA_CircularMode_t;

/**
 * @brief DMA Peripheral Flow Controller
 */
typedef enum
{
  DMA_PFCTR_DIS,  /**< Disable peripheral flow controller */
  DMA_PFCTR_EN   /**< Enable peripheral flow controller */
} DMA_PeripheralFlowCtrl_t;

/**
 * @brief DMA Transfer Complete Interrupt Enable
 */
typedef enum
{
  DMA_TCIE_DIS,  /**< Disable transfer complete interrupt */
  DMA_TCIE_EN   /**< Enable transfer complete interrupt */
} DMA_TCIE_t;

/**
 * @brief DMA Half Transfer Interrupt Enable
 */
typedef enum
{
  DMA_HTIE_DIS,  /**< Disable half transfer interrupt */
  DMA_HTIE_EN   /**< Enable half transfer interrupt */
} DMA_HTIE_t;

/**
 * @brief DMA Transfer Error Interrupt Enable
 */
typedef enum
{
  DMA_TEIE_DIS,  /**< Disable transfer error interrupt */
  DMA_TEIE_EN   /**< Enable transfer error interrupt */
} DMA_TEIE_t;

/**
 * @brief DMA Direct Mode Error Interrupt Enable
 */
typedef enum
{
  DMA_DMEIE_DIS,  /**< Disable direct mode error interrupt */
  DMA_DMEIE_EN   /**< Enable direct mode error interrupt */
} DMA_DMEIE_t;

/**
 * @brief DMA Interrupt Types
 */
typedef enum
{
  DMA_DMEIE = 1,  /**< Direct mode error interrupt */
  DMA_TEIE,       /**< Transfer error interrupt */
  DMA_HTIE,       /**< Half transfer interrupt */
  DMA_TCIE        /**< Transfer complete interrupt */
} DMA_Interrupt_t;

/**
 * @brief DMA Flag Types
 */
typedef enum
{
  DMA_FEIF,      /**< FIFO error flag */
  DMA_DMEIF = 2, /**< Direct mode error flag */
  DMA_TEIF,      /**< Transfer error flag */
  DMA_HTIF,      /**< Half transfer flag */
  DMA_TCIF       /**< Transfer complete flag */
} DMA_Flags_t;

/**
 * @brief DMA FIFO Threshold Levels
 */
typedef enum
{
  DMA_FIFO_THRESHOLD_1QUARTERFULL,  /**< FIFO 1/4 full */
  DMA_FIFO_THRESHOLD_HALFFULL,      /**< FIFO 1/2 full */
  DMA_FIFO_THRESHOLD_3QUARTERSFULL, /**< FIFO 3/4 full */
  DMA_FIFO_THRESHOLD_FULL          /**< FIFO full */
} DMA_FIFOTHRESHOLD_LEVEL_t;

/**
 * @brief DMA FIFO Mode
 */
typedef enum
{
  DMA_FIFO_MODE,    /**< FIFO mode */
  DMA_DIRECT_MODE   /**< Direct mode */
} DMA_FIFOMode_t;

/**
 * @brief DMA Configuration Structure
 * @details This structure holds all configuration parameters for DMA transfer
 */
typedef struct
{
  DMA_Stream_t              Stream;                  /**< DMA stream number (0-7) */
  DMA_Channel_t             Channel;                 /**< DMA channel number (0-7) */
  DMA_Direction_t           Direction;               /**< Transfer direction */
  DMA_PeripheralInc_t       PeripheralInc;           /**< Peripheral increment mode */
  DMA_MemoryInc_t           MemoryInc;               /**< Memory increment mode */
  DMA_PeripheralSize_t      PeriphDataAlignment;     /**< Peripheral data size */
  DMA_MemorySize_t          MemDataAlignment;        /**< Memory data size */
  DMA_CircularMode_t        CircularMode;            /**< Circular mode enable/disable */
  DMA_Priority_t            Priority;                /**< DMA priority level */
  DMA_FIFOMode_t            FIFOMode;                /**< FIFO mode enable/disable */
  DMA_FIFOTHRESHOLD_LEVEL_t FIFOTHRESHOLD_LEVEL;     /**< FIFO threshold level */
  DMA_MBURST_t              MBURST;                  /**< Memory burst transfer configuration */
  DMA_PBURST_t              PBURST;                  /**< Peripheral burst transfer configuration */
} DMA_Config_t;

/**
 * @brief DMA Handle Structure
 * @details This structure holds DMA configuration and callback functions
 */
typedef struct
{
  DMA_Stream_t              Stream;                  /**< DMA stream number */
  DMA_Channel_t             Channel;                 /**< DMA channel number */
  DMA_Priority_t            Priority;                /**< DMA priority level */
  DMA_Direction_t           Direction;               /**< Transfer direction */
  DMA_PeripheralInc_t       PeripheralInc;           /**< Peripheral increment mode */
  DMA_MemoryInc_t           MemoryInc;               /**< Memory increment mode */
  DMA_PeripheralSize_t      PeriphDataAlignment;     /**< Peripheral data size */
  DMA_MemorySize_t          MemDataAlignment;        /**< Memory data size */
  DMA_CircularMode_t        CircularMode;            /**< Circular mode enable/disable */
  DMA_FIFOMode_t            FIFOMode;                /**< FIFO mode enable/disable */
  DMA_FIFOTHRESHOLD_LEVEL_t FIFOTHRESHOLD_LEVEL;     /**< FIFO threshold level */
  DMA_MBURST_t              MBURST;                  /**< Memory burst transfer configuration */
  DMA_PBURST_t              PBURST;                  /**< Peripheral burst transfer configuration */
  DMA_DMEIE_t               DMEIE;                   /**< Direct mode error interrupt enable */
  DMA_TEIE_t                TEIE;                    /**< Transfer error interrupt enable */
  DMA_HTIE_t                HTIE;                    /**< Half transfer interrupt enable */
  DMA_TCIE_t                TCIE;                    /**< Transfer complete interrupt enable */
  void (*pfnDMECallback) (void);                    /**< Direct mode error callback */
  void (*pfnTECallback) (void);                     /**< Transfer error callback */
  void (*pfnHTCallback) (void);                     /**< Half transfer callback */
  void (*pfnTCCallback) (void);                     /**< Transfer complete callback */
} DMA_Hundle_t;

/*=================================================================================================================*/
/**
 * @fn     DMA1_enumSetConfig
 * @brief  This function configures the DMA stream
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration done successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA1_enumSetConfig(DMA_Config_t *Copy_pstConfig);

/*=================================================================================================================*/
/**
 * @fn     DMA2_enumSetConfig
 * @brief  This function configures the DMA stream
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration done successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA2_enumSetConfig(DMA_Config_t *Copy_pstConfig);

/*=================================================================================================================*/
/**
 * @fn     DMA1_enumSetConfigIT
 * @brief  This function configures the DMA stream
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration done successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA1_enumSetConfigIT(DMA_Hundle_t *Copy_pstConfig);

/*=================================================================================================================*/
/**
 * @fn     DMA2_enumSetConfigIT
 * @brief  This function configures the DMA stream
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @return ErrorState_t:
 *         - OK: Configuration done successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA2_enumSetConfigIT(DMA_Hundle_t *Copy_pstConfig);
/*=================================================================================================================*/
/**
 * @fn     DMA1_enumStartTransfer
 * @brief  This function starts the DMA transfer
 * @param  Copy_pstConfig: Pointer to the configuration structure
 * @param  Copy_u32SrcAddress: Source address
 * @param  Copy_u32DstAddress: Destination address
 * @param  Copy_u32NumOfData: Number of data to transfer
 * @return ErrorState_t:
 *         - OK: Transfer started successfully
 *         - NULL_POINTER: Invalid pointer passed to the function
 */
ErrorState_t DMA1_enumStartTransfer(DMA_Config_t *Copy_pstConfig, uint32_t* Copy_u32SrcAddress, uint32_t* Copy_u32DstAddress, uint32_t Copy_u32NumOfData);

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
ErrorState_t DMA2_enumStartTransfer(DMA_Config_t *Copy_pstConfig, uint32_t* Copy_u32SrcAddress, uint32_t* Copy_u32DstAddress, uint32_t Copy_u32NumOfData);
/*=================================================================================================================*/

/*=================================================================================================================*/

#endif /* DMA_INTERFACE_H */
