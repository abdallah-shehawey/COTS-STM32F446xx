/*
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<     CLCD_program.c     >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *
 *  Author : Abdallah Abdelmoemen Shehawey
 *  Layer  : HAL
 *  SWC    : CLCD
 *
 */

#include <SYSTIC_interface.h>

#include "STD_Macros.h"
#include "ErrTypes.h"

#include "GPIO_interface.h"

#include "CLCD_interface.h"
#include "CLCD_config.h"
#include "CLCD_private.h"
#include "CLCD_extrachar.h"

/*___________________________________________________________________________________________________________________*/
/*
###########  8 Bits Mode                                 ###########  4 Bits Mode
-------------                 ------------               -------------                 ------------
| ATmega32  |                 |   LCD    |               | ATmega32  |                 |   LCD    |
|           |                 |          |               |           |                 |          |
|        PA7|---------------->|D7        |               | PA3 or PA7|---------------->|D7        |
|        PA6|---------------->|D6        |               | PA2 or PA6|---------------->|D6        |
|        PA5|---------------->|D5        |               | PA1 or PA5|---------------->|D5        |
|        PA4|---------------->|D4        |               | PA0 or PA4|---------------->|D4        |
|        PA3|---------------->|D3        |               |           |                 |          |
|        PA2|---------------->|D2        |               |        PB2|---------------->|E         |
|        PA1|---------------->|D1        |               |        PB1|---------------->|RW        |
|        PA0|---------------->|D0        |               |        PB0|---------------->|RS        |
|           |                 |          |               |           |                 |          |
|        PB2|---------------->|E         |               |           |                 |          |
|        PB1|---------------->|RW        |               |           |                 |          |
|        PB0|---------------->|RS        |               |           |                 |          |
-----------                   ------------               -------------                 ------------
 */

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function Apply initialization sequence for LCD module
 *                                              *-------------------------------------------------------------*
 * Parameters : nothing
 * return     : nothing
 */
void CLCD_vInit(void)
{
#if CLCD_MODE == 8
  /* Initialize control pins */
  CLCD_InitPin(CLCD_CONTROL_PORT, CLCD_RS);
  CLCD_InitPin(CLCD_CONTROL_PORT, CLCD_RW);
  CLCD_InitPin(CLCD_CONTROL_PORT, CLCD_EN);

  /* Initialize data port */
  CLCD_InitPort8Bits(CLCD_DATA_PORT, CLCD_DATA_START_PIN);

  _delay_ms(50); // Must wait more than 30 ms before any action (VDD rises to 4.5v)

  CLCD_vSendCommand(CLCD_HOME);
  _delay_ms(10);

  CLCD_vSendCommand(EIGHT_BITS);
  _delay_ms(1);

  CLCD_vSendCommand(CLCD_DISPLAY_CURSOR);
  _delay_ms(1);

  CLCD_vClearScreen();

  CLCD_vSendCommand(CLCD_ENTRY_MODE);
  _delay_ms(1);

#elif CLCD_MODE == 4
  /* Initialize control pins */
  CLCD_InitPin(CLCD_CONTROL_PORT, CLCD_RS);
  CLCD_InitPin(CLCD_CONTROL_PORT, CLCD_RW);
  CLCD_InitPin(CLCD_CONTROL_PORT, CLCD_EN);

  /* Initialize data pins */
#if CLCD_DATA_NIBBLE == CLCD_HIGH_NIBBLE
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN4);
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN5);
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN6);
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN7);
#elif CLCD_DATA_NIBBLE == CLCD_LOW_NIBBLE
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN0);
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN1);
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN2);
  CLCD_InitPin(CLCD_DATA_PORT, GPIO_PIN3);
#else
#error "Wrong CLCD_DATA_NIBBLE Config"
#endif

  _delay_ms(50); // Must wait more than 30 ms before any action (VDD rises to 4.5v)

  CLCD_vSendCommand(CLCD_HOME);
  _delay_ms(10);

  CLCD_vSendCommand(FOUR_BITS);
  _delay_ms(1);

  CLCD_vSendCommand(CLCD_DISPLAY_CURSOR);
  _delay_ms(1);

  CLCD_vClearScreen();

  CLCD_vSendCommand(CLCD_ENTRY_MODE);
  _delay_ms(1);

#else
#error "Wrong CLCD_MODE Config"
#endif
}

/* Helper function to initialize a single GPIO pin */
static ErrorState_t CLCD_InitPin(GPIO_Port_t Port, GPIO_Pin_t Pin)
{
  GPIO_PinConfig_t PinConfig = {
      .Port = Port,
      .PinNum = Pin,
      .Mode = CLCD_GPIO_MODE,
      .Otype = CLCD_GPIO_OTYPE,
      .Speed = CLCD_GPIO_SPEED,
      .PullType = CLCD_GPIO_PULL};

  return GPIO_enumPinInit(&PinConfig);
}

/* Helper function to initialize 8 consecutive GPIO pins */
static ErrorState_t CLCD_InitPort8Bits(GPIO_Port_t Port, GPIO_Pin_t StartPin)
{
  GPIO_8BinsConfig_t PortConfig = {
      .Port = Port,
      .StartPin = StartPin,
      .Mode = CLCD_GPIO_MODE,
      .Otype = CLCD_GPIO_OTYPE,
      .Speed = CLCD_GPIO_SPEED,
      .PullType = CLCD_GPIO_PULL};

  return GPIO_enumPort8BitsInit(&PortConfig);
}
/*___________________________________________________________________________________________________________________*/

/*
 *         	                                      This Function send data to the port which is defined in config.h
 *                                            *------------------------------------------------------------------------*
 * Parameters :
 *		=> Copy_u8Data --> Data that you want to display (for every location )
 * return     : nothing
 */
void CLCD_vSendData(uint8_t Copy_u8Data)
{
#if CLCD_MODE == 8
  GPIO_enumWrite8BitsVal(CLCD_DATA_PORT, CLCD_DATA_START_PIN, Copy_u8Data);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RS, GPIO_PIN_HIGH);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RW, GPIO_PIN_LOW);
  CLCD_vSendFallingEdge();

#elif CLCD_MODE == 4
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RS, GPIO_PIN_HIGH);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RW, GPIO_PIN_LOW);

  GPIO_enumWrite4BitsVal(CLCD_DATA_PORT, CLCD_DATA_START_PIN, (Copy_u8Data >> 4));
  CLCD_vSendFallingEdge();
  GPIO_enumWrite4BitsVal(CLCD_DATA_PORT, CLCD_DATA_START_PIN, Copy_u8Data);
  CLCD_vSendFallingEdge();

#else
#error "Wrong CLCD_MODE Config"
#endif
}

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                       This Function Interface to send the configuration commands to the LCD Driver
 *                                                *------------------------------------------------------------------------*
 * Parameters :
 *		=> Copy_u8Command --> Command number
 * return     : nothing
 */
void CLCD_vSendCommand(uint8_t Copy_u8Command)
{
#if CLCD_MODE == 8
  GPIO_enumWrite8BitsVal(CLCD_DATA_PORT, CLCD_DATA_START_PIN, Copy_u8Command);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RS, GPIO_PIN_LOW);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RW, GPIO_PIN_LOW);
  CLCD_vSendFallingEdge();

#elif CLCD_MODE == 4
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RS, GPIO_PIN_LOW);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_RW, GPIO_PIN_LOW);

  GPIO_enumWrite4BitsVal(CLCD_DATA_PORT, CLCD_DATA_START_PIN, (Copy_u8Command >> 4));
  CLCD_vSendFallingEdge();
  GPIO_enumWrite4BitsVal(CLCD_DATA_PORT, CLCD_DATA_START_PIN, Copy_u8Command);
  CLCD_vSendFallingEdge();

#else
#error "Wrong CLCD_MODE Config"
#endif
}

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function send a pulse (falling edge ) to Enable Pin
 *                                             *-------------------------------------------------------------*
 * Parameters : nothing
 * return     : nothing
 */

static void CLCD_vSendFallingEdge(void)
{
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_EN, GPIO_PIN_HIGH);
  _delay_ms(1);
  GPIO_enumWritePinVal(CLCD_CONTROL_PORT, CLCD_EN, GPIO_PIN_LOW);
  _delay_ms(1);
}

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                     This Function clear LCD
 *                                    *-----------------------------------------------*
 * Parameters : nothing
 * return     : nothing
 */
void CLCD_vClearScreen(void)
{
  CLCD_vSendCommand(CLCD_ClEAR);
  _delay_ms(10); // wait more than 1.53 ms
}

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function send string to the port which is defined in config.h
 *                                            *------------------------------------------------------------------------*
 * Parameters :
 *		=> Copy_u8ptrString  --> Pointer to the string
 * return     : nothing
 */

void CLCD_vSendString(const uint8_t *Copy_u8PrtStrign)
{
  uint8_t LOC_u8Iterator = 0;
  while (Copy_u8PrtStrign[LOC_u8Iterator] != '\0')
  {
    CLCD_vSendData(Copy_u8PrtStrign[LOC_u8Iterator]);
    LOC_u8Iterator++;
  }
}

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function send  integer number to the port which is defined in config.h
 *                                                *----------------------------------------------------------------------------*
 * Parameters :
 *		=> Copy_s32Number  --> Number that you want to display
 * return     : nothing
 */

void CLCD_vSendIntNumber(int32_t Copy_s32Number)
{

  uint32_t LOC_u32Reverse = 1;

  if (Copy_s32Number == 0)
  {
    CLCD_vSendData('0');
  }
  else
  {
    if (Copy_s32Number < 0)
    {
      CLCD_vSendData('-');
      Copy_s32Number = (-1 * Copy_s32Number);
    }
    while (Copy_s32Number != 0)
    {
      LOC_u32Reverse = (LOC_u32Reverse * 10) + (Copy_s32Number % 10);
      Copy_s32Number /= 10;
    }
    while (LOC_u32Reverse != 1)
    {
      CLCD_vSendData((LOC_u32Reverse % 10) + 48);
      LOC_u32Reverse /= 10;
    }
  }
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function send  floating number to the port which is defined in config.h
 *                                                *----------------------------------------------------------------------------*
 * Parameters :
 *		=> Copy_f64Number  --> Number that you want to display
 * return     : nothing
 */

void CLCD_vSendFloatNumber(double Copy_f64Number)
{
  CLCD_vSendIntNumber((int32_t)Copy_f64Number);
  if (Copy_f64Number < 0)
  {
    Copy_f64Number *= -1;
  }
  Copy_f64Number = (double)Copy_f64Number - (int32_t)Copy_f64Number;
  Copy_f64Number *= 10000;
  if ((int64_t)Copy_f64Number != 0)
  {
    CLCD_vSendData('.');
    CLCD_vSendIntNumber((int32_t)Copy_f64Number);
  }
}
/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function set the cursor position
 *                                            *-------------------------------------------*
 * Parameters :
 *       => Copy_u8Row --> row number (CLCD_ROW_1 or CLCD_ROW_2)
 *		 => Copy_u8Col --> column number (CLCD_COL_1 ... CLCD_COL_16)
 * return     : nothing
 *
 * Hint       :-
 *		In This function we send a command which =0b1xxxxxxx
 *		MSB = 1  ===> refers that it is command to set cursor
 *		xxxxxxx  ===> refers to AC ( Address Counter 7Bits / DDRAM Locations 128Location )
 */

void CLCD_vSetPosition(uint8_t Copy_u8ROW, uint8_t Copy_u8Col)
{
  uint8_t LOC_u8Data;

  if ((Copy_u8ROW < CLCD_ROW_1) || (Copy_u8ROW > CLCD_ROW_4) || (Copy_u8Col < CLCD_COL_1) || (Copy_u8Col > CLCD_COL_20))
  {
    LOC_u8Data = CLCD_SET_CURSOR;
  }
  else if (Copy_u8ROW == CLCD_ROW_1)
  {
    LOC_u8Data = ((CLCD_SET_CURSOR) + (Copy_u8Col - 1));
  }
  else if (Copy_u8ROW == CLCD_ROW_2)
  {
    LOC_u8Data = ((CLCD_SET_CURSOR) + (64) + (Copy_u8Col - 1));
  }
  else if (Copy_u8ROW == CLCD_ROW_3)
  {
    LOC_u8Data = ((CLCD_SET_CURSOR) + (20) + (Copy_u8Col - 1));
  }
  else if (Copy_u8ROW == CLCD_ROW_4)
  {
    LOC_u8Data = ((CLCD_SET_CURSOR) + (84) + (Copy_u8Col - 1));
  }

  CLCD_vSendCommand(LOC_u8Data);
  _delay_ms(1);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function send extra char
 *                                                *----------------------------------*
 * Parameters :
 *      => Copy_u8Row --> row number    (CLCD_ROW_1 or CLCD_ROW_2  )
 *		=> Copy_u8Col --> column number (CLCD_COL_1 ... CLCD_COL_16)
 * return     : nothing
 *
 * Hint       :-
 *	    Address Counter can refer to CGRAM and DDRAM
 */

void CLCD_vSendExtraChar(uint8_t Copy_u8Row, uint8_t Copy_u8Col)
{

  uint8_t LOC_u8Iterator = 0;

  /* 1- Go To CGRAM            */
  CLCD_vSendCommand(CLCD_CGRAM); // Make AC refers to the first Place/Address at CGRAM

  /* 2- Draw Character in CGRAM        */
  /* Hint : it will be copied to DDRAM automatically */
  for (LOC_u8Iterator = 0; LOC_u8Iterator < (sizeof(CLCD_u8ExtraChar) / sizeof(CLCD_u8ExtraChar[0])); LOC_u8Iterator++)
  {
    CLCD_vSendData(CLCD_u8ExtraChar[LOC_u8Iterator]);
  }

  /* 3- Back (AC) To DDRAM          */
  CLCD_vSetPosition(Copy_u8Row, Copy_u8Col);

  /* 4- Send Character Address */
  for (LOC_u8Iterator = 0; LOC_u8Iterator < 8; LOC_u8Iterator++)
  {
    CLCD_vSendData(LOC_u8Iterator);
  }
}

/*___________________________________________________________________________________________________________________*/

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function shift the entire display to the right cursor follows the display shift
 *                                                *------------------------------------------------------------------------------------*
 * Parameters : nothing
 * return     : nothing
 */
void CLCD_voidShiftDisplayRight(void)
{
  CLCD_vSendCommand(CLCD_SHIFT_DISPLAY_RIGHT);
  _delay_ms(1);
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------
 *         	                                      This Function shift the entire display to the left cursor follows the display shift
 *                                                *-----------------------------------------------------------------------------------*
 * Parameters : nothing
 * return     : nothing
 */
void CLCD_voidShiftDisplayLeft(void)
{
  CLCD_vSendCommand(CLCD_SHIFT_DISPLAY_LEFT);
  _delay_ms(1);
}

/* Delay Functions Implementation */
static void _delay_us(uint32_t us)
{
  /* Calculate number of cycles needed
   * For 16MHz:
   * 1 cycle = 1/16MHz = 0.0625 us
   * We need 16 cycles for 1 us
   * Each loop iteration takes about 4 cycles
   * So we need to loop 4 times for 1 us
   */
  uint32_t cycles = us * 4;

  /* Volatile to prevent optimization */
  volatile uint32_t i;
  for (i = 0; i < cycles; i++)
  {
    __asm("NOP"); // No operation - takes 1 cycle
  }
}

static void _delay_ms(uint32_t ms)
{
  while (ms--)
  {
    _delay_us(1000); // 1000 us = 1 ms
  }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<    END    >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
