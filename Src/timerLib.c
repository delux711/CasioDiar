#include "timerLib.h"

static uint16_t timersBuff[DELAY_TIMER_END];
static bool tim_event;

void TIM2_IRQHandler() {
	TIM2->SR &= (~TIM_SR_UIF_Msk);
	tim_event = true;
}

void TIM_delayInit(void) {
	TIM_EN_delayTimers i;
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN_Msk;
	TIM2->CR1 = 0u;
	TIM2->CR1 |= TIM_CR1_DIR_Msk;
	TIM2->PSC = (TIM_FREQ_IN - 1u);
	//TIM2->ARR = 1025;
	TIM2->ARR = 1000;
	TIM2->EGR |= TIM_EGR_UG_Msk;
	TIM2->SR = 0u;
	TIM2->DIER |= TIM_DIER_UIE_Msk;
	for(i = (TIM_EN_delayTimers) 0u; i < DELAY_TIMER_END; i++) {
		timersBuff[i] = 0u;
	}
	TIM2->CR1 |= TIM_CR1_CEN_Msk;
	tim_event = false;
	NVIC_EnableIRQ(TIM2_IRQn);
}

uint16_t TIM_delayGetTime(TIM_EN_delayTimers dt) {
	return timersBuff[dt];
}

bool TIM_delayIsTimerDown(TIM_EN_delayTimers dt) {
	bool temp;
	temp = false;
	if(timersBuff[dt] == 0u) {
		temp = true;
	}
	return temp;
}

void TIM_delaySetTimer(TIM_EN_delayTimers dt, uint16_t msTime) {
	timersBuff[dt] = msTime + 1u;
}

void TIM_handleTask(void) {
	TIM_EN_delayTimers i;
	if(tim_event == true) {
		tim_event = false;
	  for(i = (TIM_EN_delayTimers) 0u; i < DELAY_TIMER_END; i++) {
		  if(timersBuff[i] != 0u) {
			  timersBuff[i]--;
			}
		}
	}
}
