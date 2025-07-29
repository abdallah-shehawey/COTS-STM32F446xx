/**
 **===========================================================================**
 **<<<<<<<<<<<<<<<<<<<<<<<<<<    LM_interface.h     >>>>>>>>>>>>>>>>>>>>>>>>>>>**
 **                                                                           **
 **                  Author : Abdallah Abdelmoemen Shehawey                   **
 **                  Layer  : HAL                                             **
 **                  CPU    : Cortex-M4                                       **
 **                  MCU    : NUCLEO-F446RE                                   **
 **                  SWC    : LED_MATRIX                                      **
 **                                                                           **
 **===========================================================================**
 */


void HLEDMATRIX_vInit();
void HLEDMATRIX_vDisplay(uint8_t *Copy_pu8Arr);