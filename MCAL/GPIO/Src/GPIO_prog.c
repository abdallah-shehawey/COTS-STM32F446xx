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

#include <stdint.h>
#include "STM32F446xx.h"

#include "STD_MACROS.h"
#include "ErrTypes.h"

#include "../Inc/GPIO_interface.h"
#include "../Inc/GPIO_private.h"
#include "../Inc/GPIO_config.h"

/* Array of GPIO port register definitions for easy access */
static GPIO_REGDEF_t *GPIO_Port[GPIO_PORT_COUNT] = {MGPIOA, MGPIOB, MGPIOC, MGPIOD, MGPIOE, MGPIOF, MGPIOG, MGPIOH};

/*=================================================================================================================*/
/**
 * @fn     GPIO_enumPinInit
 * @brief : Initializes GPIO pin configuration using array-based port access
 * @param : PinConfig[in]: Pointer to pin configuration structure containing:
 *                      - Port: Selected GPIO port index (PORTA to PORTH)
 *                      - PinNum: Selected pin number (PIN0 to PIN15)
 *                      - Mode: Pin mode (INPUT, OUTPUT, ALTFN, ANALOG)
 *                      - Otype: Output type (PUSH_PULL, OPEN_DRAIN)
 *                      - Speed: Output speed (LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED, VERY_HIGH_SPEED)
 *                      - PullType: Pull-up/Pull-down configuration (NO_PULL, PULL_UP, PULL_DOWN)
 *                      - AlternateFunction: Alternate function number if Mode is ALTFN (AF0 to AF15)
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid ptr
 */
ErrorState_t GPIO_enumPinInit(const GPIO_PinConfig_t *PinConfig)
{
  ErrorState_t Local_u8ErrorState = OK;

  if (PinConfig != NULL)
  {
    /* Check if port and pin numbers are valid */
    if ((PinConfig->Port              <=   GPIO_PORTH          ) &&
        (PinConfig->PinNum            <=   GPIO_PIN15          ) &&
        (PinConfig->Mode              <=   GPIO_ANALOG         ) &&
        (PinConfig->Otype             <=   GPIO_OPEN_DRAIN     ) &&
        (PinConfig->Speed             <=   GPIO_VERY_HIGH_SPEED) &&
        (PinConfig->PullType          <=   GPIO_PULL_DOWN      ) &&
        (PinConfig->AlternateFunction <=   GPIO_AF15           ))
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
      if ((PinConfig->Mode == GPIO_OUTPUT) || (PinConfig->Mode == GPIO_ALTFN))
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
        if (PinConfig->Mode == GPIO_ALTFN)
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
 * @fn     GPIO_enumPort8PinsInit
 * @brief : Initializes GPIO port 8-pins configuration
 * @param : GPIO_8BinsConfig[in]: Pointer to port 8-pins configuration structure containing:
 *                      - Port: Selected GPIO port (PORTA to PORTH)
 *                      - StartPin: Starting pin number (PIN0 to PIN8)
 *                      - Mode: Port mode (INPUT, OUTPUT)
 *                      - Otype: Output type (PUSH_PULL, OPEN_DRAIN)
 *                      - Speed: Output speed (LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED, VERY_HIGH_SPEED)
 *                      - PullType: Pull-up/Pull-down configuration (NO_PULL, PULL_UP, PULL_DOWN)
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid pointer
 */
ErrorState_t GPIO_enumPort8PinsInit(GPIO_8PinsConfig_t *GPIO_8PinsConfig)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint8_t Local_u8Counter;
  uint8_t Local_u8EndPin;

  if (GPIO_8PinsConfig != NULL)
  { /* Check if port and configuration parameters are valid */
    if ((GPIO_8PinsConfig->Port     <= GPIO_PORTH) &&
        (GPIO_8PinsConfig->StartPin <= GPIO_PIN15) &&
        (GPIO_8PinsConfig->Mode     <= GPIO_OUTPUT) &&
        (GPIO_8PinsConfig->Otype    <= GPIO_OPEN_DRAIN) &&
        (GPIO_8PinsConfig->Speed    <= GPIO_VERY_HIGH_SPEED) &&
        (GPIO_8PinsConfig->PullType <= GPIO_PULL_DOWN))
    {
      Local_u8EndPin = GPIO_8PinsConfig->StartPin + 7;

      /* Configure the 8 pins */
      for (Local_u8Counter = GPIO_8PinsConfig->StartPin; Local_u8Counter <= Local_u8EndPin; Local_u8Counter++)
      {
        GPIO_8PinsConfig->StartPin = Local_u8Counter;
        GPIO_enumPinInit((GPIO_PinConfig_t*) GPIO_8PinsConfig); //Error when casting removed
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
 * @fn     GPIO_enumPort4PinsInit
 * @brief : Initializes GPIO port 4-pins configuration
 * @param : Port4PinsConfig[in]: Pointer to port 4-pins configuration structure containing:
 *                      - Port: Selected GPIO port (PORTA to PORTH)
 *                      - StartPin: Starting pin number (PIN0 to PIN15)
 *                      - Mode: Port mode (INPUT, OUTPUT)
 *                      - Otype: Output type (PUSH_PULL, OPEN_DRAIN)
 *                      - Speed: Output speed (LOW_SPEED, MEDIUM_SPEED, HIGH_SPEED, VERY_HIGH_SPEED)
 *                      - PullType: Pull-up/Pull-down configuration (NO_PULL, PULL_UP, PULL_DOWN)
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid pointer
 */
ErrorState_t GPIO_enumPort4PinsInit(GPIO_4PinsConfig_t *GPIO_4PinsConfig)
{
  ErrorState_t Local_u8ErrorState = OK;
  uint8_t Local_u8Counter;
  uint8_t Local_u8EndPin;

  if (GPIO_4PinsConfig != NULL)
  { /* Check if port and configuration parameters are valid */
    if ((GPIO_4PinsConfig->Port <= GPIO_PORTH) &&
        (GPIO_4PinsConfig->StartPin <= GPIO_PIN15) &&
        (GPIO_4PinsConfig->Mode <= GPIO_OUTPUT) &&
        (GPIO_4PinsConfig->Otype <= GPIO_OPEN_DRAIN) &&
        (GPIO_4PinsConfig->Speed <= GPIO_VERY_HIGH_SPEED) &&
        (GPIO_4PinsConfig->PullType <= GPIO_PULL_DOWN))
    {
      Local_u8EndPin = GPIO_4PinsConfig->StartPin + 3;

      /* Configure the 4 pins */
      for (Local_u8Counter = GPIO_4PinsConfig->StartPin; Local_u8Counter <= Local_u8EndPin; Local_u8Counter++)
      {
        GPIO_4PinsConfig->StartPin = Local_u8Counter;
        GPIO_enumPinInit((GPIO_PinConfig_t*) GPIO_4PinsConfig);
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
 * @fn     GPIO_enumWrite4BitsVal
 * @brief : Write 4 bits directly to ODR register starting from specified pin
 * @param : Port: GPIO port index (PORTA to PORTH)
 * @param : StartPin: Starting pin number (PIN0 to PIN12)
 * @param : Value: 4-bit value to write (0x0 to 0xF)
 * @retval ErrorState_t: OK if write successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumWrite4PinsVal(GPIO_Port_t Port, GPIO_Pin_t StartPin, uint8_t Value)
{
  ErrorState_t Local_u8ErrorState = OK;

  if ((Port <= GPIO_PORTH) && (StartPin <= GPIO_PIN15) && (Value <= 0x0F))
  {
    // Clear the 4 bits
    GPIO_Port[Port]->ODR &= ~(0x0F << StartPin);
    // Write the new value
    GPIO_Port[Port]->ODR |= ((Value & 0x0F) << StartPin);
  }
  else
  {
    Local_u8ErrorState = NOK;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     GPIO_enumWrite8BitsVal
 * @brief : Write 8 bits directly to ODR register starting from specified pin
 * @param : Port: GPIO port index (PORTA to PORTH)
 * @param : StartPin: Starting pin number (PIN0 to PIN15)
 * @param : Value: 8-bit value to write (0x00 to 0xFF)
 * @retval ErrorState_t: OK if write successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumWrite8PinsVal(GPIO_Port_t Port, GPIO_Pin_t StartPin, uint8_t Value)
{
  ErrorState_t Local_u8ErrorState = OK;

  if ((Port <= GPIO_PORTH) && (StartPin <= GPIO_PIN8) && (Value <= 0xFF))
  {
    GPIO_Port[Port]->ODR &= ~(0xFF << StartPin);
    GPIO_Port[Port]->ODR |= ((Value & 0xFF) << StartPin);
  }
  else
  {
    Local_u8ErrorState = NOK;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     GPIO_enumWritePinVal
 * @brief : Write a value to a specific GPIO pin using array-based port access
 * @param : Port: GPIO port index (PORTA to PORTH)
 * @param : PinNum: Selected pin number (PIN0 to PIN15)
 * @param : PinVal: Value to write (PIN_LOW or PIN_HIGH)
 * @retval ErrorState_t: OK if write successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumWritePinVal(GPIO_Port_t Port, GPIO_Pin_t PinNum, GPIO_PinValue_t PinVal)
{
  ErrorState_t Local_u8ErrorState = OK;

  /* Check if port and pin numbers are valid */
  if ((Port <= GPIO_PORTH) && (PinNum <= GPIO_PIN15) && (PinVal == GPIO_PIN_HIGH || PinVal == GPIO_PIN_LOW))
  {
    /* Write the value to the pin using BSRR register for atomic access */
    switch (PinVal)
    {
    case GPIO_PIN_HIGH:
      GPIO_Port[Port]->BSRR = (1u << PinNum); /* Set bits are in the low half of BSRR */
      /*GPIO_Port[Port]->ODR |= (1u << PinNum);*/
      break;
    case GPIO_PIN_LOW:
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
 * @fn     GPIO_enumReadPinVal
 * @brief : Read the current value of a specific GPIO pin using array-based port access
 * @param : Port: GPIO port index (PORTA to PORTH)
 * @param : PinNum: Selected pin number (PIN0 to PIN15)
 * @param : PinVal: Pointer to store the read value (PIN_LOW or PIN_HIGH)
 * @retval ErrorState_t: OK if read successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumReadPinVal(GPIO_Port_t Port, GPIO_Pin_t PinNum, GPIO_PinValue_t *PinVal)
{
  ErrorState_t Local_u8ErrorState = OK;
  if (PinVal != NULL)
  {
    /* Check if port and pin numbers are valid and PinVal pointer is not NULL */
    if ((Port <= GPIO_PORTH) && (PinNum <= GPIO_PIN15))
    {
      /* Read the pin value by checking the corresponding bit in IDR register */
      *PinVal = (GPIO_Port[Port]->IDR & (1u << PinNum)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
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
 * @fn     GPIO_enumTogPinVal
 * @brief : Toggle the current value of a specific GPIO pin using array-based port access
 * @param : Port: GPIO port index (PORTA to PORTH)
 * @param : PinNum: Selected pin number (PIN0 to PIN15)
 * @retval ErrorState_t: OK if toggle successful, NOK if invalid parameters
 */
ErrorState_t GPIO_enumTogPinVal(GPIO_Port_t Port, GPIO_Pin_t PinNum)
{
  ErrorState_t Local_u8ErrorState = OK;

  /* Check if port and pin numbers are valid */
  if ((Port >= GPIO_PORTA && Port <= GPIO_PORTH) &&(PinNum >= GPIO_PIN0 && PinNum <= GPIO_PIN15))
  {
    GPIO_Port[Port]->ODR ^= (1 << PinNum);
  }
  else
  {
    Local_u8ErrorState = NOK;
  }
  return Local_u8ErrorState;
}

/*=================================================================================================================*/
/**
 * @fn     GPIO_enumRead4PinsVal
 * @brief : Read the current value of a GPIO port 4-pins
 * @param : Port: GPIO port (PORTA to PORTH)
 * @param : StartPin: Starting pin number (PIN0 to PIN15)
 * @param : PinsValue: Pointer to store the read value
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid pointer
 */
ErrorState_t GPIO_enumRead4PinsVal(GPIO_Port_t Port, GPIO_Pin_t StartPin, uint8_t *PinsValue)
{
  ErrorState_t Local_u8ErrorState = OK;
  if (PinsValue != NULL)
  {
    if ((Port <= GPIO_PORTH) && (StartPin <= GPIO_PIN15))
    {
      *PinsValue = (GPIO_Port[Port]->IDR & (0x0F << StartPin));
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
 * @fn     GPIO_enumRead8PinsVal
 * @brief : Read the current value of a GPIO port 8-pins
 * @param : Port: GPIO port (PORTA to PORTH)
 * @param : StartPin: Starting pin number (PIN0 to PIN15)
 * @param : PinsValue: Pointer to store the read value
 * @retval ErrorState_t: OK if configuration successful, NOK if invalid parameters, NULL_POINTER if invalid pointer
 */
ErrorState_t GPIO_enumRead8PinsVal(GPIO_Port_t Port, GPIO_Pin_t StartPin, uint8_t *PinsValue)
{
  ErrorState_t Local_u8ErrorState = OK;
  if (PinsValue != NULL)
  {
    if ((Port <= GPIO_PORTH) && (StartPin <= GPIO_PIN15))
    {
      *PinsValue = (GPIO_Port[Port]->IDR & (0xFF << StartPin));
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
