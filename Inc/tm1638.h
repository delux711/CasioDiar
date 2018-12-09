#ifndef _TM1638_H
#define _TM1638_H
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx.h"

/*
  Byte0        Byte1  Byte2 Byte3 Byte4 Byte5 Byte6 Byte7 Byte8 Byte9 Byte10 Byte11 Byte12 Byte13 Byte14 Byte15
  Seg1         Led1   Seg2  Led2  Seg3  Led3  Seg4  Led4  Seg5  Led5  Seg6   Led6   Seg7   Led7   Seg8   Led8
HSB    MSB  HSB    MSB	(1-LEDy)
 XGFEDCBA    000000001
----------------------------------------------------------------------------
|  LED1     LED2     LED3     LED4     LED5     LED6     LED7     LED8     |
|                                                                          |
|  --A--    --A--    --A--    --A--    --A--    --A--    --A--    --A--    |
|  |   |    |   |    |   |    |   |    |   |    |   |    |   |    |   |    |
|  F   B    F   B    F   B    F   B    F   B    F   B    F   B    F   B    |
|  --G--    --G--    --G--    --G--    --G--    --G--    --G--    --G--    |
|  |   |    |   |    |   |    |   |    |   |    |   |    |   |    |   |    |
|  E   C    E   C    E   C    E   C    E   C    E   C    E   C    E   C    |
|  --D-- X  --D-- X  --D-- X  --D-- X  --D-- X  --D-- X  --D-- X  --D-- X  |
----------------------------------------------------------------------------
   SEG1     SEG2     SEG3     SEG4     SEG5     SEG6     SEG7     SEG8
*/

#define TM1638_SET_ADDRESS			(0xC0u)

#define TM1638_COMMAND_WRITE_DATA	(0x40u)
#define TM1638_COMMAND_READ_DATA	(0x42u)
#define TM1638_COMMAND_ADDRESS_AUTO	(0x43u)

#define TM1638_COMMAND_PULSE_1_16	(0x80u)
#define TM1638_COMMAND_PULSE_2_16	(0x81u)
#define TM1638_COMMAND_PULSE_4_16	(0x82u)
#define TM1638_COMMAND_PULSE_10_16	(0x83u)
#define TM1638_COMMAND_PULSE_11_16	(0x84u)
#define TM1638_COMMAND_PULSE_12_16	(0x85u)
#define TM1638_COMMAND_PULSE_13_16	(0x86u)
#define TM1638_COMMAND_PULSE_14_16	(0x87u)
#define TM1638_COMMAND_LCD_OFF		(0x80u)
#define TM1638_COMMAND_LCD_ON		(0x8Fu)

extern void tm1638_init(void);
extern void tm1638_show(uint8_t *buff);
extern void tm1638_showPos(uint8_t position, uint8_t ch);

#endif
