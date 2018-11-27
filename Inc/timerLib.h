#ifndef __timerLib_H
#define __timerLib_H

#include "stm32l4xx.h"                  // Device header
#include <stdbool.h>

#define TIM_FREQ_IN		(4u)		// input frequency to TIM2

typedef enum {
	DELAY_TIMER_CASIO_DIAR,
	DELAY_TL_HORE,
	DELAY_TIMER_TEST,
	DELAY_TIMER_END
} TIM_EN_delayTimers;

extern void TIM_delayInit(void);
extern void TIM_handleTask(void);
extern uint16_t TIM_delayGetTime(TIM_EN_delayTimers dt);
extern bool TIM_delayIsTimerDown(TIM_EN_delayTimers dt);
extern void TIM_delaySetTimer(TIM_EN_delayTimers dt, uint16_t msTime);

#endif
