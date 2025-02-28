/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    GPIO_interface.h   >>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : MCAL                                            **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : GPIO                                            **
 **                                                                           **
 **===========================================================================**
 */

#ifndef GPIO_INTERFACE_H_
#define GPIO_INTERFACE_H_

/************************** Port Definitions **************************/
typedef enum
{
  PORTA = 0, /* GPIO Port A */
  PORTB,     /* GPIO Port B */
  PORTC,     /* GPIO Port C */
  PORTD,     /* GPIO Port D */
  PORTE,     /* GPIO Port E */
  PORTF,     /* GPIO Port F */
  PORTG,     /* GPIO Port G */
  PORTH      /* GPIO Port H */
} Port_t;

/************************** Pin Definitions **************************/
typedef enum
{
  PIN0 = 0, /* GPIO Pin 0 */
  PIN1,     /* GPIO Pin 1 */
  PIN2,     /* GPIO Pin 2 */
  PIN3,     /* GPIO Pin 3 */
  PIN4,     /* GPIO Pin 4 */
  PIN5,     /* GPIO Pin 5 */
  PIN6,     /* GPIO Pin 6 */
  PIN7,     /* GPIO Pin 7 */
  PIN8,     /* GPIO Pin 8 */
  PIN9,     /* GPIO Pin 9 */
  PIN10,    /* GPIO Pin 10 */
  PIN11,    /* GPIO Pin 11 */
  PIN12,    /* GPIO Pin 12 */
  PIN13,    /* GPIO Pin 13 */
  PIN14,    /* GPIO Pin 14 */
  PIN15     /* GPIO Pin 15 */
} Pin_t;

/************************** Mode Definitions **************************/
typedef enum
{
  INPUT = 0, /* Input Mode */
  OUTPUT,    /* Output Mode */
  ALTFN,     /* Alternate Function Mode */
  ANALOG     /* Analog Mode */
} Mode_t;

/************************** Output Type Definitions **************************/
typedef enum
{
  PUSH_PULL = 0, /* Push-Pull Output Type */
  OPEN_DRAIN     /* Open-Drain Output Type */
} OutputType_t;

/************************** Output Speed Definitions **************************/
typedef enum
{
  LOW_SPEED = 0,  /* Low Speed */
  MEDIUM_SPEED,   /* Medium Speed */
  HIGH_SPEED,     /* High Speed */
  VERY_HIGH_SPEED /* Very High Speed */
} OutputSpeed_t;

/************************** Pull-up/Pull-down Definitions **************************/
typedef enum
{
  NO_PULL = 0, /* No Pull-up or Pull-down */
  PULL_UP,     /* Pull-up */
  PULL_DOWN    /* Pull-down */
} PullUpDown_t;

/************************** Alternate Function Definitions **************************/
typedef enum
{
  AF0 = 0, /* Alternate Function 0 */
  AF1,     /* Alternate Function 1 */
  AF2,     /* Alternate Function 2 */
  AF3,     /* Alternate Function 3 */
  AF4,     /* Alternate Function 4 */
  AF5,     /* Alternate Function 5 */
  AF6,     /* Alternate Function 6 */
  AF7,     /* Alternate Function 7 */
  AF8,     /* Alternate Function 8 */
  AF9,     /* Alternate Function 9 */
  AF10,    /* Alternate Function 10 */
  AF11,    /* Alternate Function 11 */
  AF12,    /* Alternate Function 12 */
  AF13,    /* Alternate Function 13 */
  AF14,    /* Alternate Function 14 */
  AF15     /* Alternate Function 15 */
} AlternateFunction_t;

/************************** Pin Value Definitions **************************/
typedef enum
{
  PIN_LOW = 0, /* Pin Low Value */
  PIN_HIGH     /* Pin High Value */
} PinValue_t;

/************************** Pin Configuration Structure **************************/
typedef struct
{
  Port_t Port;                           /* GPIO Port Selection */
  Pin_t PinNum;                          /* GPIO Pin Number */
  Mode_t Mode;                           /* GPIO Mode */
  OutputType_t Otype;                    /* Output Type (if configured as output) */
  OutputSpeed_t Speed;                   /* Output Speed (if configured as output) */
  PullUpDown_t PullType;                 /* Pull-up/Pull-down Configuration */
  AlternateFunction_t AlternateFunction; /* Alternate Function (if in AF mode) */
} PinConfig_t;

/************************** Function Prototypes **************************/
/**
 * @fn     GPIO_enumPinInit
 * @brief  Initializes GPIO pin configuration
 * @param  PinConfig[in]: Pointer to pin configuration structure
 * @retval GPIO_ErrorState: GPIO_OK if successful, GPIO_NOK if error
 */
ErrorState_t GPIO_enumPinInit(const PinConfig_t *PinConfig);

/**
 * @brief  Write a value to a GPIO pin
 * @param  Port: GPIO port (PORTA to PORTH)
 * @param  PinNum: Pin number (PIN0 to PIN15)
 * @param  PinVal: Value to write (PIN_LOW or PIN_HIGH)
 * @retval GPIO_ErrorState: GPIO_OK if successful, GPIO_NOK if error
 */
ErrorState_t GPIO_enumWritePinVal(Port_t Port, Pin_t PinNum, PinValue_t PinVal);

/**
 * @brief  Read the current value of a GPIO pin
 * @param  Port: GPIO port (PORTA to PORTH)
 * @param  PinNum: Pin number (PIN0 to PIN15)
 * @param  PinVal: Pointer to store the read value
 * @retval GPIO_ErrorState: GPIO_OK if successful, GPIO_NOK if error
 */
ErrorState_t GPIO_enumReadPinVal(Port_t Port, Pin_t PinNum, PinValue_t *PinVal);

/**
 * @brief  Toggle the current value of a GPIO pin
 * @param  Port: GPIO port (PORTA to PORTH)
 * @param  PinNum: Pin number (PIN0 to PIN15)
 * @retval GPIO_ErrorState: GPIO_OK if successful, GPIO_NOK if error
 */
ErrorState_t GPIO_enumTogPinVal(Port_t Port, Pin_t PinNum);

#endif /* GPIO_INTERFACE_H_ */
