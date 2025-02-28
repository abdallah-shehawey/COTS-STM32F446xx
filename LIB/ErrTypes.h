/********************************************************
 * @file ErrTypes.h
 * @name Abdallah AbdelMomen
 * @brief Return Error State types
 */
#ifndef ERRTYPES_H_
#define ERRTYPES_H_

#define NULL    0

#define ENABLE  1
#define DISABLE 0

/* Func States  */
#define IDLE 0
#define BUSY 1


typedef enum
{
  OK = 0,
  NOK,
  NULL_POINTER,
  BUSY_STATE,
  TIMEOUT_STATE
} ErrorState_t;

#endif /* ERRTYPES_H_ */
