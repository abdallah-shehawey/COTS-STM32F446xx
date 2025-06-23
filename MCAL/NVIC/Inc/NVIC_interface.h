/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    NVIC_interface.h    >>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : NVIC                                            **
 **                                                                           **
 **===========================================================================**
 */
#ifndef NVIC_INTERFACE_H_
#define NVIC_INTERFACE_H_

#include "stdint.h"

#define NVIC_WWDGEN 0
#define NVIC_PVDEN 1
#define NVIC_TAMP_STAMP 2
#define NVIC_RTC_WAKEUP 3
#define NVIC_FLASH 4
#define NVIC_RCC 5
#define NVIC_EXTI0 6
#define NVIC_EXTI1 7
#define NVIC_EXTI2 8
#define NVIC_EXTI3 9
#define NVIC_EXTI4 10
#define NVIC_DMA1_STREAM0 11
#define NVIC_DMA1_STREAM1 12
#define NVIC_DMA1_STREAM2 13
#define NVIC_DMA1_STREAM3 14
#define NVIC_DMA1_STREAM4 15
#define NVIC_DMA1_STREAM5 16
#define NVIC_DMA1_STREAM6 17
#define NVIC_ADC 18
#define NVIC_CAN1_TX 19
#define NVIC_CAN1_RX0 20
#define NVIC_CAN1_RX1 21
#define NVIC_CAN1_SCE 22
#define NVIC_EXTI9_5 23
#define NVIC_TIM1_BRK_TIM9 24
#define NVIC_TIM1_UP_TIM10 25
#define NVIC_TIM1_TRG_COM_TIM11 26
#define NVIC_TIM1_CC 27
#define NVIC_TIM2 28
#define NVIC_TIM3 29
#define NVIC_TIM4 30
#define NVIC_I2C1_EV 31
#define NVIC_I2C1_ER 32
#define NVIC_I2C2_EV 33
#define NVIC_I2C2_ER 34
#define NVIC_SPI1 35
#define NVIC_SPI2 36
#define NVIC_USART1 37
#define NVIC_USART2 38
#define NVIC_USART3 39
#define NVIC_EXTI15_10 40
#define NVIC_RTC_ALARM 41
#define NVIC_OTG_FS_WKUP 42
#define NVIC_DMA1_STREAM7 47
#define NVIC_SDIO 49
#define NVIC_TIM5 50
#define NVIC_SPI3 51
#define NVIC_DMA2_STREAM0 56
#define NVIC_DMA2_STREAM1 57
#define NVIC_DMA2_STREAM2 58
#define NVIC_DMA2_STREAM3 59
#define NVIC_DMA2_STREAM4 60
#define NVIC_OTG_FS 67
#define NVIC_DMA2_STREAM5 68
#define NVIC_DMA2_STREAM6 69
#define NVIC_DMA2_STREAM7 70
#define NVIC_USART6 71
#define NVIC_I2C3_EV 72
#define NVIC_I2C3_ER 73
#define NVIC_FPU 81
#define NVIC_SPI4 84
#define NVIC_SPI5 85
#define NVIC_QUADSPI 92
#define NVIC_FMPI2C1_EV 95
#define NVIC_FMPI2C1_ER 96

ErrorState_t NVIC_vEnableIRQ(uint8_t Copy_u8IRQNumber);
ErrorState_t NVIC_vDisableIRQ(uint8_t Copy_u8IRQNumber);
ErrorState_t NVIC_vSetPendingFlag(uint8_t Copy_u8IRQNumber);
ErrorState_t NVIC_vClearPendingFlag(uint8_t Copy_u8IRQNumber);
ErrorState_t NVIC_vGetActiveFlag(uint8_t Copy_u8IRQNumber,   uint8_t *Copy_pu8Flag);
ErrorState_t NVIC_vSetPriority(uint8_t Copy_u8IRQNumber, uint8_t Copy_u8Priority);

#endif /* NVIC_INTERFACE_H_ */
