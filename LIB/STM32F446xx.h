/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    STM32F446xx.h     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : LIB                                             **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : STM32F446xx                                     **
 **                                                                           **
 **===========================================================================**
 */

#ifndef STM32F446xx_H
#define STM32F446xx_H

/**************************************         Various Memories Base Adresses          ******************************************/
#define FLASH_BASEADDR 0x08000000UL
#define SRAM_BASEADDR  0x20000000UL
#define ROM_BASEADDR   0x1FFF0000UL

/**************************************         AHB1 Peripheral Base Adresses          ******************************************/
#define GPIOA_BASEADDR 0X40020000UL
#define GPIOB_BASEADDR 0X40020400UL
#define GPIOC_BASEADDR 0X40020800UL
#define GPIOD_BASEADDR 0X40020C00UL
#define GPIOE_BASEADDR 0X40021000UL
#define GPIOF_BASEADDR 0X40021400UL
#define GPIOG_BASEADDR 0X40021800UL
#define GPIOH_BASEADDR 0X40021C00UL

#define RCC_BASEADDR   0x40023800UL

/**************************************         AHB2 Peripheral Base Adresses          ******************************************/
/**************************************         AHB3 Peripheral Base Adresses          ******************************************/
/**************************************         APB1 Peripheral Base Adresses          ******************************************/
/**************************************         APB2 Peripheral Base Adresses          ******************************************/
/**************************************         APB3 Peripheral Base Adresses          ******************************************/

/**************************************       GPIO Register Definition Structure       ******************************************/
typedef struct
{
  volatile uint32_t MODER;   /* GPIO PORT mode register              */
  volatile uint32_t OTYPER;  /* GPIO PORT output type register       */
  volatile uint32_t OSPEEDR; /* GPIO PORT output speed register      */
  volatile uint32_t PUPDR;   /* GPIO PORT pull-up/pull-down register */
  volatile uint32_t IDR;     /* GPIO PORT input data register        */
  volatile uint32_t ODR;     /* GPIO PORT output data register       */
  volatile uint32_t BSRR;    /* GPIO PORT bit set/reset register     */
  volatile uint32_t LCKR;    /* GPIO PORT configuration lock register*/
  volatile uint32_t AFR[2];  /* GPIO alternate function low register */
} GPIO_REGDEF_t;

/**************************************       RCC Register Definitions Structure       ******************************************/
typedef struct
{
  volatile uint32_t CR;           /* RCC clock control register                                                 */
  volatile uint32_t PLLCFGR;      /* RCC PLL configuration register (RCC_PLLCFGR)                               */
  volatile uint32_t CFGR;         /* RCC clock configuration register (RCC_CFGR)                                */
  volatile uint32_t CIR;          /* RCC clock interrupt register (RCC_CIR)                                     */
  volatile uint32_t AHP1RSTR;     /* RCC AHB1 peripheral reset register (RCC_AHB1RSTR)                          */
  volatile uint32_t AHP2RSTR;     /* RCC AHB2 peripheral reset register (RCC_AHB2RSTR)                          */
  volatile uint32_t AHP3RSTR;     /* RCC AHB3 peripheral reset register (RCC_AHB3RSTR)                          */
  volatile uint32_t RESERVED1[1]; /* RESERVED                                                                   */
  volatile uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register (RCC_APB1RSTR)                          */
  volatile uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register (RCC_APB2RSTR)                          */
  volatile uint32_t RESERVED2[2]; /* RESERVED                                                                   */
  volatile uint32_t AHP1ENR;      /* RCC AHB1 peripheral clock enable register (RCC_AHB1ENR)                    */
  volatile uint32_t AHP2ENR;      /* RCC AHB2 peripheral clock enable register (RCC_AHB2ENR)                    */
  volatile uint32_t AHP3ENR;      /* RCC AHB3 peripheral clock enable register (RCC_AHB3ENR)                    */
  volatile uint32_t RESERVED3[1]; /* RESERVED                                                                   */
  volatile uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register (RCC_APB1ENR)                    */
  volatile uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register (RCC_APB2ENR)                    */
  volatile uint32_t RESERVED4[2]; /* RESERVED                                                                   */
  volatile uint32_t AHB1LPENR;    /* RCC AHB1 peripheral clock enable in low power mode register(RCC_AHB1LPENR) */
  volatile uint32_t AHP2LPENR;    /* RCC AHB2 peripheral clock enable in low power mode register(RCC_AHB2LPENR) */
  volatile uint32_t AHP3LPENR;    /* RCC AHB3 peripheral clock enable in low power mode register(RCC_AHB3LPENR) */
  volatile uint32_t RESERVED5[1]; /* RESERVED                                                                   */
  volatile uint32_t APB1LPENR;    /* RCC APB1 peripheral clock enable in low power mode register(RCC_APB1LPENR) */
  volatile uint32_t APB2LPENR;    /* RCC APB2 peripheral clock enabled in low power mode register(RCC_APB2LPENR)*/
  volatile uint32_t RESERVED6[2]; /* RESERVED                                                                   */
  volatile uint32_t BDCR;         /* RCC Backup domain control register (RCC_BDCR)                              */
  volatile uint32_t CSR;          /* RCC clock control & status register (RCC_CSR)                              */
  volatile uint32_t RESERVED7[2]; /* RESERVED                                                                   */
  volatile uint32_t SSCGR;        /* RCC spread spectrum clock generation register (RCC_SSCGR)                  */
  volatile uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register (RCC_PLLI2SCFGR)                         */
  volatile uint32_t PLLSAICFGR;   /* RCC PLL configuration register (RCC_PLLSAICFGR)                            */
  volatile uint32_t DCKCFGR;      /* RCC dedicated clock configuration register (RCC_DCKCFGR)                   */
  volatile uint32_t CKGATENR;     /* RCC clocks gated enable register (CKGATENR)                                */
  volatile uint32_t DCKCFGR2;     /* RCC dedicated clocks configuration register 2 (DCKCFGR2)                   */
} RCC_RegDef_t;

/**************************************         GPIO Peripheral Definitions       ******************************************/

#define MGPIOA ((GPIO_REGDEF_t *)GPIOA_BASEADDR)
#define MGPIOB ((GPIO_REGDEF_t *)GPIOB_BASEADDR)
#define MGPIOC ((GPIO_REGDEF_t *)GPIOC_BASEADDR)
#define MGPIOD ((GPIO_REGDEF_t *)GPIOD_BASEADDR)
#define MGPIOE ((GPIO_REGDEF_t *)GPIOE_BASEADDR)
#define MGPIOF ((GPIO_REGDEF_t *)GPIOF_BASEADDR)
#define MGPIOG ((GPIO_REGDEF_t *)GPIOG_BASEADDR)
#define MGPIOH ((GPIO_REGDEF_t *)GPIOH_BASEADDR)

/**************************************         RCC Peripheral Definitions       *********************************************/

#define MRCC ((RCC_RegDef_t *)RCC_BASEADDR)

#endif /* STM32F446xx_H */
