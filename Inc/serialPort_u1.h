#ifndef __serialPort_u1_H
#define __serialPort_u1_H

#include "stm32l4xx.h"                  // Device header
#include <stdbool.h>

extern void SPu1_init(void);
extern bool SPu1_isNewData(void);
extern void SPu1_pauseOn(void);
extern void SPu1_pauseOff(void);
extern uint8_t SPu1_getData(void);
extern uint8_t SPu1_sendChar(uint8_t ch);
extern uint8_t SPu1_sendBuff(uint8_t *buff, uint8_t length);
extern uint8_t SPu1_sendString(uint8_t *buff);

#endif
