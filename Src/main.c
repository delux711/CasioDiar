//#include "Board_LED.h"                  // ::Board Support:LED
//#include "Board_Buttons.h"              // ::Board Support:Buttons
#include "getJoystick.h"
#include "lcd.h"
#include "myRTC.h"
#include "serialPort.h"
#include "casioDiar.h"
#include "timerLib.h"
#include <stdio.h>

// SYSCLK - 4MHz
// MSI - 4MHz
// HSI16 - 16MHz
// 100: HSE clock selected - OFF
// 101: Main PLL clock selected - OFF
// 110: LSI clock selected - 32kHz
// 111: LSE clock selected - OFF
/*
static uint64_t *lcdRam1;
static uint64_t *lcdRam2;
static uint64_t *lcdRam3;
static uint64_t *lcdRam4;
static uint8_t data[] = { 'A', 255, 255, 255, 255, 255 };*/
static uint8_t testDiar[] = { "Pavol Pusztai, Slovinska 1, 05342 Krompachy." };
int main(void) {
	char buff[14];
	uint32_t count = 0;
  uint8_t stredTmp = 0;
	/*
	LED_Initialize();
	Buttons_Initialize();
	LED_On(0);
	LED_Off(0);
	*/
	tl_Init();
	TIM_delayInit();
	//LED_On(0);
	//LED_Off(0);
	MX_LCD_Init();
	LCD_GLASS_Clear();
   myRtcInit();
		RCC->CFGR |= (4u << RCC_CFGR_MCOPRE_Pos); // MCO / 16 IF 4
	SP_init();
	//LCD_GLASS_DisplayString((uint8_t *)"A");
	/*	LCD_GLASS_DisplayString(data);
	lcdRam1 = (uint64_t*) &LCD->RAM[0];
	lcdRam2 = (uint64_t*) &LCD->RAM[2];
	lcdRam3 = (uint64_t*) &LCD->RAM[4];
	lcdRam4 = (uint64_t*) &LCD->RAM[6];
	if(*lcdRam1 && *lcdRam2 && *lcdRam3 && *lcdRam4) {
	}*/
	printf("\nTestovanie\n");
	sprintf(buff, "%06d", count);
	LCD_GLASS_DisplayString((uint8_t*) buff);
	while(1) {
        myRtcLcd();
        CD_task();
        TIM_handleTask();
		if(tl_getTlSample().hore) {
			led1_on();
			CD_sendToDiarConst(0u);
		} else {
			led1_off();
		}
        if(tl_getTlSample().lavo) {
            myRtcGetTime((uint8_t *)buff);
            sprintf(&buff[6], "-%06d", ++count);
            CD_sendToDiarConst((uint8_t *)buff);
		}
        if(tl_getTlSample().pravo) {
            CD_sendToDiarConst(testDiar);
		}
        if(tl_getTl().dole) {
			led2_on();
			CD_senToDiarEndCommunication();
		} else {
			led2_off();
		}
		if(tl_getTl().stred) {
			stredTmp = 1;
			sprintf(buff, "%06d", ++count);
			LCD_GLASS_DisplayString((uint8_t*) buff);
		} else if (stredTmp) {
			if(USART2->ISR & USART_ISR_TXE) {
				stredTmp = 0;
				printf("Count: %d", count);
			}
		}
	}
}
