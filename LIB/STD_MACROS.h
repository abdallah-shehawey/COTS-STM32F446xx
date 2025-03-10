/*
 *<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<    STD_MACROS.h    >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
 *
 *  Author : Abdallah Abdelmoemen Shehawey
 *  Layer  : LIB
 *
 *
 */
#ifndef STD_MACROS_h_
#define STD_MACROS_h_

#define REGISTER_SIZE 8
#define SET_BIT(reg, bit)     reg |= (1 << bit)
#define CLR_BIT(reg, bit)     reg &= (~(1 << bit))
#define TOG_BIT(reg, bit)     reg ^= (1 << bit)
#define READ_BIT(reg, bit)    (reg & (1 << bit)) >> bit

#define READ_BYTE(reg, byte)  (reg & (0XFF << (byte * 8))) >> (byte * 8)

#define IS_BIT_SET(reg, bit)  (reg & (1 << bit)) >> bit
#define IS_BIT_CLR(reg, bit)  !((reg & (1 << bit)) >> bit)

#define ROR(reg, num)         reg = (reg << (REGISTER_SIZE - num)) | (reg >> (num))
#define ROL(reg, num)         reg = (reg >> (REGISTER_SIZE - num)) | (reg << (num))

#endif /* STD_MACROS_H_ */