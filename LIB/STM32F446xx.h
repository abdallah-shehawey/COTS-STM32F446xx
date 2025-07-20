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
#define FLASH_BASEADDR  0x08000000UL
#define SRAM_BASEADDR   0x20000000UL
#define ROM_BASEADDR    0x1FFF0000UL

/**************************************         NVIC Base Adresses          ******************************************/
#define NVIC_BASEADDR   0XE000E100UL

/**************************************         SCB Base Adresses          ******************************************/
#define SCB_BASEADDR    0XE000ED00UL

/**************************************         AHB1 Peripheral Base Adresses          ******************************************/
#define GPIOA_BASEADDR  0X40020000UL
#define GPIOB_BASEADDR  0X40020400UL
#define GPIOC_BASEADDR  0X40020800UL
#define GPIOD_BASEADDR  0X40020C00UL
#define GPIOE_BASEADDR  0X40021000UL
#define GPIOF_BASEADDR  0X40021400UL
#define GPIOG_BASEADDR  0X40021800UL
#define GPIOH_BASEADDR  0X40021C00UL

#define RCC_BASEADDR    0x40023800UL

#define SYSTIC_BASEADDR 0XE000E010UL

/*Internal DMA Base Adresses */
#define DMA1_BASEADDR   0X40026000UL
#define DMA2_BASEADDR   0X40026400UL

/**************************************         AHB2 Peripheral Base Adresses          ******************************************/
/**************************************         AHB3 Peripheral Base Adresses          ******************************************/
/**************************************         APB1 Peripheral Base Adresses          ******************************************/
/**************************************         APB2 Peripheral Base Adresses          ******************************************/

#define SYSCFG_BASEADDR 0X40013800UL
#define EXTI_BASEADDR   0X40013C00UL

/**************************************         APB3 Peripheral Base Adresses          ******************************************/

/**************************************         SYSTIC Peripheral Definitions       *********************************************/

typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile uint32_t CALIB;
} SYSTIC_RegDef_t;

#define MSYSTIC ((SYSTIC_RegDef_t *)SYSTIC_BASEADDR)

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


/**************************************       DMA Regster Definitions Structure       ******************************************/
typedef struct
{
  uint32_t CR;       // Stream x configuration register (DMA_SxCR)
  uint32_t NDTR;     // Stream x number of data register (DMA_SxNDTR)
  uint32_t PAR;      // Stream x peripheral address register (DMA_SxPAR)
  uint32_t M0AR;     // Stream x memory 0 address register (DMA_SxM0AR)
  uint32_t M1AR;     // Stream x memory 1 address register (DMA_SxM1AR)
  uint32_t FCR;      // Stream x FIFO control register (DMA_SxFCR)
}DMA_STREAM_REGDEF_t;

typedef struct
{
  uint32_t LISR;      // Low interrupt status register (DMA_LISR)
  uint32_t HISR;      // High interrupt status register (DMA_HISR)
  uint32_t LIFCR;     // Low interrupt flag clear register (DMA_LIFCR)
  uint32_t HIFCR;     // High interrupt flag clear register (DMA_HIFCR)
  DMA_STREAM_REGDEF_t Stream[8];
}DMA_REGDEF_t;

#define MDMA1 ((DMA_REGDEF_t *)DMA1_BASEADDR)
#define MDMA2 ((DMA_REGDEF_t *)DMA2_BASEADDR)

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

/**************************************         NVIC Peripheral Definitions       *********************************************/
typedef struct
{
  volatile uint32_t ISER[8];        /* Interrupt Set Enable Register */
  volatile uint32_t RESERVED1[24];
  volatile uint32_t ICER[8];        /* Interrupt Clear Enable Register */
  volatile uint32_t RESERVED2[24];
  volatile uint32_t ISPR[8];        /* Interrupt Set Pending Register */
  volatile uint32_t RESERVED3[24];
  volatile uint32_t ICPR[8];        /* Interrupt Clear Pending Register */
  volatile uint32_t RESERVED4[24];
  volatile uint32_t IABR[8];        /* Interrupt Active Bit Register */
  volatile uint32_t RESERVED5[56];
  volatile uint8_t  IPR[240];        /* Interrupt Priority Register */
  volatile uint32_t RESERVED6[580];
  volatile uint32_t STIR;           /* Software Trigger Interrupt Register */
} NVIC_RegDef_t;

#define MNVIC ((NVIC_RegDef_t *)NVIC_BASEADDR)

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

/**************************************         SCB Peripheral Definitions       *********************************************/

typedef struct
{
  uint32_t CPUID;     // CPU Identification Register
  uint32_t ICSR;      // Interrupt Control and State Register
  uint32_t VTOR;      // Vector Table Offset Register
  uint32_t AIRCR;     // Application Interrupt and Reset Control Register
  uint32_t SCR;       // System Control Register
  uint32_t CCR;       // Configuration and Control Register
  uint32_t SHPR1;     // System Handler Priority Register 1 (Priority of SVCall)
  uint32_t SHPR2;     // System Handler Priority Register 2 (Priority of Debug Monitor)
  uint32_t SHPR3;     // System Handler Priority Register 3 (Priority of PendSV and SysTick)
  uint32_t SHCSR;     // System Handler Control and State Register
  uint8_t  CFSR;      // Configurable Fault Status Register (lower 8 bits)
  uint8_t  BFSR;      // BusFault Status Register (lower 8 bits)
  uint16_t UFSR;      // UsageFault Status Register (lower 16 bits)
  uint32_t HFSR;      // HardFault Status Register
  uint32_t DFSR;      // Debug Fault Status Register
  uint32_t MMAR;      // MemManage Fault Address Register
  uint32_t BFAR;      // BusFault Address Register
  uint32_t AFSR;      // Auxiliary Fault Status Register
} SCB_RegDef_t;

#define MSCB ((SCB_RegDef_t *)SCB_BASEADDR)


/**************************************         SYSCFG Peripheral Definitions       *********************************************/

typedef struct
{
  uint32_t MEMRMP;    // Memory Remap Register
  uint32_t PMC;       // PMC Register
  uint32_t EXTICR[4]; // External Interrupt Configuration Register
  uint32_t Reserved1[2];
  uint32_t CMPCR;     // Compensation Cell Control Register
  uint32_t Reserved2[2];
  uint32_t CFGR;
} SYSCFG_RegDef_t;

#define MSYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

/**************************************         EXTI Peripheral Definitions       *********************************************/
typedef struct
{
  uint32_t IMR;       // Interrupt Mask Register
  uint32_t EMR;       // Event Mask Register
  uint32_t RTSR;      // Rising Trigger Selection Register
  uint32_t FTSR;      // Falling Trigger Selection Register
  uint32_t SWIER;     // Software Interrupt Event Register
  uint32_t PR;        // Pending Register
} EXTI_RegDef_t;

#define MEXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#endif /* STM32F446xx_H */
