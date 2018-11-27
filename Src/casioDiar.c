#include "casioDiar.h"
#include "serialPort.h"
#include "serialPort_u1.h"
#include "timerLib.h"
#include "getJoystick.h"
#include <stdio.h>

#define CD_TIMEOUT_DEF	(1000000u)
#define CD_MAX_NOT_ANSWERE  (5u)

/*
typedef enum _cd_waitChar_e {
	CD_WAIT_CHAR_WAITING,
	CD_WAIT_CHAR_RECEIVED,
	CD_WAIT_CHAR_ERROR
} cd_waitChar;
*/

typedef enum _cd_state_e {
	CD_STATE_NOT_INIT,
	CD_STATE_SLEEP,
	CD_STATE_SENDING_CR,		// 0x0D
	CD_STATE_SENDING_LF,		// 0x0A
	CD_STATE_SENDING_COLON,	// dvojbodka
	CD_STATE_RECEIVING_CR,
	CD_STATE_RECEIVING_WAIT_CR,
	CD_STATE_RECEIVING_LF,
	CD_STATE_RECEIVING_WAIT_FOR_0X11,
	CD_STATE_RECEIVING_0X11	// white for 0x11 from diar
} cd_state_e;

static cd_state_e cd_state = CD_STATE_NOT_INIT;
static uint32_t cd_timeout = CD_TIMEOUT_DEF;
static uint8_t cd_buffer[50];
static uint8_t cd_countNotAns = 0u;

bool bWaitForCharacter(uint8_t ch);
bool bIsReceivedCharacter(uint8_t ch);
bool cd_test = false;

void CD_task(void) {
	if(cd_test != false) {	
		if(TIM_delayIsTimerDown(DELAY_TIMER_TEST) == true) {
			HDIO_testPinOff();
			cd_test = false;
		}
	}
		
	if(cd_state == CD_STATE_NOT_INIT) {
		cd_state = CD_STATE_SLEEP;
		SP_init();
		TIM_delayInit();
		SPu1_init();

				HDIO_testPinInit();
        HDIO_testPinOff();
				TIM_delaySetTimer(DELAY_TIMER_TEST, 3u);
				//cd_test = true;

	} else if(cd_state == CD_STATE_SLEEP) {
	
	} else if(cd_state == CD_STATE_SENDING_CR) {
		if(bWaitForCharacter('\r') == true) {
			cd_state = CD_STATE_SENDING_LF;
		}
	} else if(cd_state == CD_STATE_SENDING_LF) {
		if(bWaitForCharacter('\n') == true) {
			cd_state = CD_STATE_SENDING_COLON;
		}
	} else if(cd_state == CD_STATE_SENDING_COLON) {
		if(bWaitForCharacter('\n') == true) {
			cd_state = CD_STATE_SLEEP;
		}
	} else if(cd_state == CD_STATE_RECEIVING_CR) {
        if(cd_countNotAns < CD_MAX_NOT_ANSWERE) {
					HDIO_testPinOn();
            SPu1_sendChar(0x0D);    // 0x0D = \r
					HDIO_testPinOff();
            cd_state = CD_STATE_RECEIVING_WAIT_CR;
            TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, 5u);
        } else {
            cd_countNotAns = 0u;
            cd_state = CD_STATE_SLEEP;
        }
	} else if(cd_state == CD_STATE_RECEIVING_WAIT_CR) {
		if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
			cd_state = CD_STATE_RECEIVING_LF;
		}
	} else if(cd_state == CD_STATE_RECEIVING_LF) {
		GPIOD->ODR &= (~GPIO_ODR_OD0_Msk); // pin OFF
		SPu1_sendChar(0x0A);    // 0x0A = \n
		cd_state = CD_STATE_RECEIVING_WAIT_FOR_0X11;
		TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, 50u);
	} else if(cd_state == CD_STATE_RECEIVING_WAIT_FOR_0X11) {
		if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
      cd_countNotAns++;
			cd_state = CD_STATE_RECEIVING_CR;
		} else {
      if(bIsReceivedCharacter(0x11) == true) {
				cd_state = CD_STATE_SLEEP;  // next state
        }
      }
	} else if(cd_state == CD_STATE_RECEIVING_0X11) {
		
	}
}

uint8_t CD_receive(void) {
	cd_state = CD_STATE_SENDING_CR;
	return cd_buffer[0];
}

void CD_sendToDiarConst(void) {
	if(TIM_delayIsTimerDown(DELAY_TL_HORE) == true) {
		if(cd_state == CD_STATE_SLEEP) {
			cd_countNotAns = 0u;
			TIM_delaySetTimer(DELAY_TL_HORE, 500);
			cd_state = CD_STATE_RECEIVING_CR;
		}
	}
}

bool bWaitForCharacter(uint8_t ch) {
	bool bReturn;
	bReturn = false;
	if(cd_timeout == 0) {
		cd_state = CD_STATE_SLEEP;
		cd_timeout = CD_TIMEOUT_DEF;
	} else {
		cd_timeout--;
		if(SP_isNewData() == true) {
			if(SP_getData() == ch) {
				cd_timeout = CD_TIMEOUT_DEF;
				bReturn = true;
			}
		}
	}
	return bReturn;
}

bool bIsReceivedCharacter(uint8_t ch) {
    bool ret;
    ret = false;
    if(SPu1_isNewData() == true) {
        if(SPu1_getData() == ch) {
            ret = true;
        }
    }
    return ret;
}
