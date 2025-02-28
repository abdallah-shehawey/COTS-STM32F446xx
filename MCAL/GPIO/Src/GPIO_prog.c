/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    GPIO_prog.c     >>>>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : GPIO                                            **
 **                                                                           **
 **===========================================================================**
 */

#include <ErrTypes.h>
#include <stdint.h>
#include "STM32F446xx.h"

#include "GPIO_interface.h"
#include "GPIO_private.h"
#include "GPIO_config.h"

#include "STD_MACROS.h"
#include "ErrTypes.h"

/* Array of GPIO port register definitions for easy access */
static GPIO_REGDEF_t *GPIO_Port[GPIO_PORT_COUNT] = {MGPIOA, MGPIOB, MGPIOC, MGPIOD, MGPIOE, MGPIOF, MGPIOG, MGPIOH};

/*=================================================================================================================*/
/**
 * @brief  Initializes GPIO pin configuration using array-based port access
 * @param  PinConfig[in]: Pointer to pin configuration structure containing:
 *                      - Port: Selected GPIO port index (PORTA to PORTH)
 *                      - PinNum: Selected pin number (PIN0 to PIN15)
 *                      - Mode: Pin mode (INPUT, OUTPUT, ALTFN, ANALOG)
 *                      - Otype: Output type (PUSH_PULL, OPEN_DRAIN)
 *                      - Speed: Output speed (LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED, VERY_HIGH_SPEED)
 *                      - PullType: Pull-up/Pull-down configuration (NO_PULL, PULL_UP, PULL_DOWN)
 *                      - AlternateFunction: Alternate function number if Mode is ALTFN (AF0 to AF15)
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid ptr
 */
ErrorState_t GPIO_enumPinInit(const PinConfig_t *PinConfig)
{
  ErrorState_t Local_u8ErrorState = OK;

  if (PinConfig != NULL)
  {
    /* Check if port and pin numbers are valid */
    if ((PinConfig->Port <= PORTH) &&
        (PinConfig->PinNum <= PIN15) &&
        (PinConfig->Mode <= ANALOG) &&
        (PinConfig->Otype <= OPEN_DRAIN) &&
        (PinConfig->Speed <= VERY_HIGH_SPEED) &&
        (PinConfig->PullType <= PULL_DOWN) &&
        (PinConfig->AlternateFunction <= AF15))
    {
      /* Configure pin mode (Input/Output/Alternate Function/Analog) */
      (GPIO_Port[PinConfig->Port]->MODER) &= ~(MODER_MASK << ((PinConfig->PinNum) * MODER_PIN_ACCESS));
      (GPIO_Port[PinConfig->Port]->MODER) |= (PinConfig->Mode << ((PinConfig->PinNum) * MODER_PIN_ACCESS));
      //_______________________________________________________________________________________________________________//
      /* Set Pull up or Pull down or No pull */
      // if the pin mode is analog, HW will ignore the pull type
      (GPIO_Port[PinConfig->Port]->PUPDR) &= ~(PUPDR_MASK << ((PinConfig->PinNum) * PUPDR_PIN_ACCESS));
      (GPIO_Port[PinConfig->Port]->PUPDR) |= (PinConfig->PullType << ((PinConfig->PinNum) * PUPDR_PIN_ACCESS));
      //_______________________________________________________________________________________________________________//
      /* select output type and output speed in case of general purpose output or alternate function */
      if ((PinConfig->Mode == OUTPUT) || (PinConfig->Mode == ALTFN))
      {
        /* Set output type : push pull or open drain */
        GPIO_Port[PinConfig->Port]->OTYPER &= ~(OTYPER_MASK << PinConfig->PinNum);
        GPIO_Port[PinConfig->Port]->OTYPER |= (PinConfig->Otype << PinConfig->PinNum);
        //_______________________________________________________________________________________________________________//
        /* Configure output speed : low, medium, high, fast */
        GPIO_Port[PinConfig->Port]->OSPEEDR &= ~(OSPEEDR_MASK << (PinConfig->PinNum * OSPEEDR_PIN_ACCESS));
        GPIO_Port[PinConfig->Port]->OSPEEDR |= (PinConfig->Speed << (PinConfig->PinNum * OSPEEDR_PIN_ACCESS));
        //_______________________________________________________________________________________________________________//
        /* select the pin alternate function */
        if (PinConfig->Mode == ALTFN)
        {
          GPIO_Port[PinConfig->Port]->AFR[(PinConfig->PinNum) / AFR_PIN_SHIFT] &= ~(AFR_MASK << (((PinConfig->PinNum) % 8u) * AFR_PIN_ACCESS));
          GPIO_Port[PinConfig->Port]->AFR[(PinConfig->PinNum) / AFR_PIN_SHIFT] |= (PinConfig->AlternateFunction << (((PinConfig->PinNum) % 8u) * AFR_PIN_ACCESS));
        }
      }
    }
    else
    {
      Local_u8ErrorState = NOK;
    }
  }
  else
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief  Initializes GPIO port half configuration (8 pins)
 * @param  PortHalfConfig[in]: Pointer to port half configuration structure containing:
 *                      - Port: Selected GPIO port (PORTA to PORTH)
 *                      - PortHalf: Port half selection (PORT_FIRST_HALF for pins 0-7, PORT_SECOND_HALF for pins 8-15)
 *                      - Mode: Port mode (INPUT, OUTPUT)
 *                      - Otype: Output type (PUSH_PULL, OPEN_DRAIN)
 *                      - Speed: Output speed (LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED, VERY_HIGH_SPEED)
 *                      - PullType: Pull-up/Pull-down configuration (NO_PULL, PULL_UP, PULL_DOWN)
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid pointer
 */
ErrorState_t GPIO_enumPortHalfInit(const GPIO_PortHalfConfig_t *PortHalfConfig)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint8_t Local_u8Counter;
  uint8_t Local_u8StartPin;
  uint8_t Local_u8EndPin;

  if (PortHalfConfig != NULL)
  { /* Check if port and configuration parameters are valid */
    if ((PortHalfConfig->Port <= PORTH) &&
        (PortHalfConfig->Mode <= OUTPUT) &&
        (PortHalfConfig->Otype <= OPEN_DRAIN) &&
        (PortHalfConfig->Speed <= VERY_HIGH_SPEED) &&
        (PortHalfConfig->PullType <= PULL_DOWN))
    {
      /* Determine start and end pins based on port half selection */
      if (PortHalfConfig->PortHalf == PORT_FIRST_HALF)
      {
        Local_u8StartPin = 0;
        Local_u8EndPin = 7;
      }
      else
      {
        Local_u8StartPin = 8;
        Local_u8EndPin = 15;
      }

      /* Configure the 8 pins */
      for (Local_u8Counter = Local_u8StartPin; Local_u8Counter <= Local_u8EndPin; Local_u8Counter++)
      {
        /* Set pin mode */
        GPIO_Port[PortHalfConfig->Port]->MODER &= ~(MODER_MASK << (Local_u8Counter * MODER_PIN_ACCESS));
        GPIO_Port[PortHalfConfig->Port]->MODER |= (PortHalfConfig->Mode << (Local_u8Counter * MODER_PIN_ACCESS));

        /* Set pull type */
        GPIO_Port[PortHalfConfig->Port]->PUPDR &= ~(PUPDR_MASK << (Local_u8Counter * PUPDR_PIN_ACCESS));
        GPIO_Port[PortHalfConfig->Port]->PUPDR |= (PortHalfConfig->PullType << (Local_u8Counter * PUPDR_PIN_ACCESS));

        /* Configure output settings if output or alternate function mode */
        if (PortHalfConfig->Mode == OUTPUT)
        {
          /* Set output type */
          GPIO_Port[PortHalfConfig->Port]->OTYPER &= ~(OTYPER_MASK << Local_u8Counter);
          GPIO_Port[PortHalfConfig->Port]->OTYPER |= (PortHalfConfig->Otype << Local_u8Counter);

          /* Set output speed */
          GPIO_Port[PortHalfConfig->Port]->OSPEEDR &= ~(OSPEEDR_MASK << (Local_u8Counter * OSPEEDR_PIN_ACCESS));
          GPIO_Port[PortHalfConfig->Port]->OSPEEDR |= (PortHalfConfig->Speed << (Local_u8Counter * OSPEEDR_PIN_ACCESS));
        }
      }
    }
    else
    {
      Local_u8ErrorState = NOK;
    }
  }
  else
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief  Write a value to a specific GPIO pin using array-based port access
 * @param  Port: GPIO port index (PORTA to PORTH)
 * @param  PinNum: Selected pin number (PIN0 to PIN15)
 * @param  PinVal: Value to write (PIN_LOW or PIN_HIGH)
 * @retval ErrorState_t: OK if write successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumWritePinVal(Port_t Port, Pin_t PinNum, PinValue_t PinVal)
{
  ErrorState_t Local_u8ErrorState = OK;

  /* Check if port and pin numbers are valid */
  if ((Port <= PORTH) && (PinNum <= PIN15) && (PinVal == PIN_HIGH || PinVal == PIN_LOW))
  {
    /* Write the value to the pin using BSRR register for atomic access */
    switch (PinVal)
    {
    case PIN_HIGH:
      GPIO_Port[Port]->BSRR = (1u << PinNum); /* Set bits are in the low half of BSRR */
      /*GPIO_Port[Port]->ODR |= (1u << PinNum);*/
      break;
    case PIN_LOW:
      GPIO_Port[Port]->BSRR = (1u << (PinNum + 16u)); /* Reset bits are in the high half of BSRR */
      /*GPIO_Port[Port]->ODR &= ~(1u << PinNum);*/
      break;
    default:
      Local_u8ErrorState = NOK;
      break;
    }
  }
  else
  {
    Local_u8ErrorState = NOK;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief  Read the current value of a specific GPIO pin using array-based port access
 * @param  Port: GPIO port index (PORTA to PORTH)
 * @param  PinNum: Selected pin number (PIN0 to PIN15)
 * @param  PinVal: Pointer to store the read value (PIN_LOW or PIN_HIGH)
 * @retval ErrorState_t: OK if read successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumReadPinVal(Port_t Port, Pin_t PinNum, PinValue_t *PinVal)
{
  ErrorState_t Local_u8ErrorState = OK;
  if (PinVal != NULL)
  {
    /* Check if port and pin numbers are valid and PinVal pointer is not NULL */
    if ((Port <= PORTH) && (PinNum <= PIN15))
    {
      /* Read the pin value by checking the corresponding bit in IDR register */
      *PinVal = (GPIO_Port[Port]->IDR & (1u << PinNum)) ? PIN_HIGH : PIN_LOW;
    }
    else
    {
      Local_u8ErrorState = NOK;
    }
  }
  else
  {
    Local_u8ErrorState = NULL_POINTER;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @brief  Toggle the current value of a specific GPIO pin using array-based port access
 * @param  Port: GPIO port index (PORTA to PORTH)
 * @param  PinNum: Selected pin number (PIN0 to PIN15)
 * @retval ErrorState_t: OK if toggle successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumTogPinVal(Port_t Port, Pin_t PinNum)
{
  ErrorState_t Local_u8ErrorState = OK;

  /* Check if port and pin numbers are valid */
  if ((Port >= PORTA && Port <= PORTH) &&
      (PinNum >= PIN0 && PinNum <= PIN15))
  {
    TOG_BIT(GPIO_Port[Port]->ODR, PinNum);
  }
  else
  {
    Local_u8ErrorState = NOK;
  }
  return Local_u8ErrorState;
}
