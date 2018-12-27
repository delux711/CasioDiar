#ifndef _TM1638_H
#define _TM1638_H
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx.h"

/* This pin is output */
#define TM_STB_OUT()     TM_STB_OUT_MAP(E,13)
#define TM_STB_0()       TM_STB_0_MAP(E,13)
#define TM_STB_1()       TM_STB_1_MAP(E,13)

/* This pin is output */
#define TM_CLK_OUT()     TM_CLK_OUT_MAP(E,14)
#define TM_CLK_0()       TM_CLK_0_MAP(E,14)
#define TM_CLK_1()       TM_CLK_1_MAP(E,14)

/* This pin is output/input with internal pull up resistor and open collector */
#define TM_DIO_CONFIG()  TM_DIO_CONFIG_MAP(E,15)
#define TM_DIO_IN()      TM_DIO_IN_MAP(E,15)
#define TM_DIO_OUT()     TM_DIO_OUT_MAP(E,15)
#define TM_DIO_DATA()    TM_DIO_DATA_MAP(E,15)
#define TM_DIO_0()       TM_DIO_0_MAP(E,15)
#define TM_DIO_1()       TM_DIO_1_MAP(E,15)

/*
  Byte0        Byte1  Byte2 Byte3 Byte4 Byte5 Byte6 Byte7 Byte8 Byte9 Byte10 Byte11 Byte12 Byte13 Byte14 Byte15
  Seg1         Led1   Seg2  Led2  Seg3  Led3  Seg4  Led4  Seg5  Led5  Seg6   Led6   Seg7   Led7   Seg8   Led8
HSB    MSB  HSB    MSB    (1-LEDy)
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

#define TM1638_MAX_DONE_COUNT   (20u)

typedef enum _TM1638_status_tl {
    TM1638_STATUS_TL_NOT_INIT,
    TM1638_STATUS_TL_NOT_INIT2,
    TM1638_STATUS_TL_WRITE,
    /* Must be together! */
    TM1638_STATUS_TL_READ1,
    TM1638_STATUS_TL_READ2,
    TM1638_STATUS_TL_READ3,
    TM1638_STATUS_TL_READ4,
    TM1638_STATUS_TL_READ_DONE,
    /* Must be together! */
    TM1638_STATUS_TL_CALC,
    TM1638_STATUS_TL_DONE
} TM1638_status_tl;


#define TM1638_SET_ADDRESS                   (0xC0u)

#define TM1638_COMMAND_WRITE_DATA            (0x40u)
#define TM1638_COMMAND_READ_DATA             (0x42u)
#define TM1638_COMMAND_ADDRESS_AUTO          (0x43u)

#define TM1638_COMMAND_LCD_ON_PULSE_1_16     (0x88u)
#define TM1638_COMMAND_LCD_ON_PULSE_2_16     (0x89u)
#define TM1638_COMMAND_LCD_ON_PULSE_4_16     (0x8Au)
#define TM1638_COMMAND_LCD_ON_PULSE_10_16    (0x8Bu)
#define TM1638_COMMAND_LCD_ON_PULSE_11_16    (0x8Cu)
#define TM1638_COMMAND_LCD_ON_PULSE_12_16    (0x8Du)
#define TM1638_COMMAND_LCD_ON_PULSE_13_16    (0x8Eu)
#define TM1638_COMMAND_LCD_ON_PULSE_14_16    (0x8Fu)
#define TM1638_COMMAND_LCD_OFF               (0x80u)
#define TM1638_COMMAND_LCD_ON_MAX            (0x8Fu)

extern TM1638_status_tl TM1638_handleTaskTl(void);
extern uint8_t tm1638_getTl(void);
extern void tm1638_init(void);
extern void tm1638_sendCommand(uint8_t comm);
extern void tm1638_sendPacket(uint8_t *buff, uint8_t size);
extern void tm1638_show(uint8_t *buff);
extern void tm1638_showPos(uint8_t position, uint8_t ch);
extern void tm1638_readTl(uint8_t *buff);

#endif
