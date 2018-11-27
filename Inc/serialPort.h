#ifndef __serialPort_H
#define __serialPort_H

#include "stm32l4xx.h"                  // Device header
#include <stdbool.h>

extern void SP_init(void);
extern bool SP_isNewData(void);
extern uint8_t SP_getData(void);

#endif
