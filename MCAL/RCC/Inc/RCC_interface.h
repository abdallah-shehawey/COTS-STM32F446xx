/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<   RCC_interface.h   >>>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : RCC                                             **
 **                                                                           **
 **===========================================================================**
 */

#ifndef MCAL_RCC_INTERFACE_H_
#define MCAL_RCC_INTERFACE_H_

/* Define Bus in uC*/
typedef enum
{
  AHB1,
  AHB2,
  AHB3,
  APB1,
  APB2,
} uC_BUS_t;

/* Define peripheral at AHB1 BUS */
typedef enum
{
  GPIOAEN,
  GPIOBEN,
  GPIOCEN,
  GPIODEN,
  GPIOEEN,
  GPIOFEN,
  GPIOGEN,
  GPIOHEN,
  CRCEN = 12,
  BKPSRAMEN = 18,
  DMA1EN = 21,
  DMA2EN,
  OTGHSEN = 29,
  OTGHSULPIEN,
} AHB1_BUS_t;

/* Define peripheral at AHB2 BUS */
typedef enum
{
  DCMIEN,
  OTGFSEN = 7,
} AHB2_BUS_t;

/* Define peripheral at AHB3 BUS */
typedef enum
{
  RCC_FMCEN,
  RCC_QSPIEN,
} AHB3_BUS_t;

/* Define peripheral at APB2 BUS */
typedef enum
{
  TIM1EN,
  TIM8EN,
  USART1 = 4,
  USART6,
  ADC1EN = 8,
  ADC2EN,
  ADC3EN,
  SDIOEN,
  SPI1EN,
  SPI4EN,
  SYSCFGEN,
  TIM9EN = 16,
  TIM10EN,
  TIM11EN,
  SAI1EN = 22,
  SAI2EN,
} APB2_BUS_t;

/* Define peripheral at APB1 BUS */
typedef enum
{
  TIM2EN,
  TIM3EN,
  TIM4EN,
  TIM5EN,
  TIM6EN,
  TIM7EN,
  TIM12EN,
  TIM13EN,
  TIM14EN,
  WWDGEN = 11,
  SPI2EN = 14,
  SPI3EN,
  SPDIFRX,
  USART2EN,
  USART3EN,
  USART4EN,
  USART5EN,
  I2C1EN,
  I2C2EN,
  I2C3EN,
  FMPI2C1EN,
  CAN1EN,
  CAN2EN,
  CECEN,
  PWREN,
  DACEN,
} APB1_BUS_t;

/* Define Division Factor To AHP */
typedef enum
{
  AHB_NOT_DIV = 7,
  AHB_DIV_2,
  AHB_DIV_4,
  AHB_DIV_8,
  AHB_DIV_16,
  AHB_DIV_64,
  AHB_DIV_128,
  AHB_DIV_256,
  AHB_DIV_512,
} AHB_BUS_DIV_t;

/* Define Division Factor To APB */
typedef enum
{
  APB_NOT_DIV = 3,
  APB_DIV_2,
  APB_DIV_4,
  APB_DIV_8,
  APB_DIV_16,
} APB_BUS_DIV_t;

/* Define division factor for the main PLL */
#define MPLL_DIV_2 2
#define MPLL_DIV_63 63

/* Define Multiplication factor to NPLL */
#define NPLL_CLK_MULT_50 50
#define NPLL_CLK_MULT_432 432

/* define division factor to PLLCLK */
typedef enum
{
  PPLL_DIV_2,
  PPLL_DIV_4,
  PPLL_DIV_6,
  PPLL_DIV_8,
} PPLL_CLK_DIV;

/* Define Clock Source To PLL */
typedef enum
{
  PLL_HSI,
  PLL_HSE,
} PLL_CLK_SRC;

/* Define type of CLK source */
typedef enum
{
  HSI_CLK,
  HSE_CLK,
  PLLP_CLK,
  PLLR_CLK,
  PLL_CLK,
} CLK_SRC_t;

/* Define ON or OFF for CLK */
typedef enum
{
  CLK_ON,
  CLK_OFF,
} CLK_EN_t;

/* Define ON or OFF for Peripheral */
typedef enum
{
  PER_ON,
  PER_OFF,
} PER_EN_t;

typedef enum
{
  RCC_OK = 0, /* Operation completed successfully */
  RCC_NOK,    /* Operation failed */
} RCC_ErrorState;

/**
 * @brief   Structure for PLL configuration parameters
 */
typedef struct
{
  uint8_t PLLSource; /* PLL clock source (PLL_HSI, PLL_HSE) */
  uint8_t PLLM_Div;  /* PLL M divider (MPLL_DIV_2 to MPLL_DIV_63) */
  uint8_t PLLN_Mult; /* PLL N multiplier (NPLL_CLK_MULT_50 to NPLL_CLK_MULT_432) */
  uint8_t PLLP_Div;  /* PLL P divider (PPLL_DIV_2 to PPLL_DIV_8) */
} RCC_PLLConfig_t;

/* functions defination */
RCC_ErrorState RCC_enumSetClkSts(uint8_t Copy_u8CLK, uint8_t Copy_u8Status);
RCC_ErrorState RCC_enumSetSysClk(uint8_t Copy_u8CLK);
RCC_ErrorState RCC_enumPLLConfig(const RCC_PLLConfig_t *Copy_PLLConfig);
RCC_ErrorState RCC_enumAHBConfig(uint8_t Copy_u8AHPDiv);
RCC_ErrorState RCC_enumAPB1Config(uint8_t Copy_u8APB1Div);
RCC_ErrorState RCC_enumAPB2Config(uint8_t Copy_u8APB2Div);
RCC_ErrorState RCC_enumAHPPerSts(uint8_t Copy_u8Bus, uint8_t Copy_u8AHPPer, uint8_t Copy_u8Status);
RCC_ErrorState RCC_enumABPPerSts(uint8_t Copy_u8Bus, uint8_t Copy_u8AHPPer, uint8_t Copy_u8Status);

#endif /* MCAL_RCC_INTERFACE_H_ */
