#ifndef __serialPort_H
#define __serialPort_H

#include "stm32l4xx.h"                  // Device header
#include <stdbool.h>

extern void SP_init(void);
extern bool SP_isNewData(void);
extern void SP_pauseOn(void);
extern void SP_pauseOff(void);
extern uint8_t SP_getData(void);
extern uint8_t SP_sendChar(uint8_t ch);
extern uint8_t SP_sendBuff(uint8_t *buff, uint8_t length);
extern uint8_t SP_sendString(uint8_t *buff);

#endif
