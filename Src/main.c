//#include "Board_LED.h"                  // ::Board Support:LED
//#include "Board_Buttons.h"              // ::Board Support:Buttons
#include "getJoystick.h"
#include "lcd.h"
#include "myRTC.h"
#include "serialPort.h"
#include "casioDiar.h"
#include "timerLib.h"
#include "serialPort_u1.h"
#include <stdio.h>
#include <stdbool.h>
#include "BMP180_pressure.h"

// SYSCLK - 4MHz
// MSI - 4MHz
// HSI16 - 16MHz
// 100: HSE clock selected - OFF
// 101: Main PLL clock selected - OFF
// 110: LSI clock selected - 32kHz
// 111: LSE clock selected - OFF

void run_in_ram(void);
void load_ramcode(void);
extern char Image$$RW_CODE$$Base;		// kvoli testovaniu volania funkcie z RAM
extern char Image$$RW_CODE$$Length; // kvoli testovaniu volania funkcie z RAM
extern char Load$$RW_CODE$$Base;		// kvoli testovaniu volania funkcie z RAM
__attribute__((section("TEMPDATASECTION"), zero_init)) uint8_t foo;

/*
static uint64_t *lcdRam1;
static uint64_t *lcdRam2;
static uint64_t *lcdRam3;
static uint64_t *lcdRam4;
static uint8_t data[] = { 'A', 255, 255, 255, 255, 255 };*/
static uint8_t testDiar[] = { "Ing.Pavol Pusztai, Slovinska 1, 05342 Krompachy." };
static uint8_t receiveBuff[50];
static uint8_t receiveP = 0;
int main(void) {
    bool lcdIsShift;
	bool cdNewData = false;
    uint8_t i, ch;
	uint8_t buff[50];
    uint8_t cd_buff[300];
    uint32_t count = 0;
    uint8_t stredTmp = 0;
    
	SPu1_init();
	
    load_ramcode();
    tl_Init();
    TIM_delayInit();
    MX_LCD_Init();
    LCD_GLASS_Clear();
    myRtcInit();
    RCC->CFGR |= (4u << RCC_CFGR_MCOPRE_Pos); // MCO / 16 IF 4
    SP_init();

    count = 0u;

    //LCD_GLASS_DisplayString((uint8_t *)"A");
    /*	LCD_GLASS_DisplayString(data);
    lcdRam1 = (uint64_t*) &LCD->RAM[0];
    lcdRam2 = (uint64_t*) &LCD->RAM[2];
    lcdRam3 = (uint64_t*) &LCD->RAM[4];
    lcdRam4 = (uint64_t*) &LCD->RAM[6];
    if(*lcdRam1 && *lcdRam2 && *lcdRam3 && *lcdRam4) {
    }*/
    printf("\nTestovanie\n");
    sprintf((char*)buff, "%06d", count);
    LCD_GLASS_DisplayString(buff);
    if(tl_getTl().stred) {
		i = 0u;
		for(ch = ('a'-1U); ch < ('z'+1u); ch++) {
			buff[i++] = ch;
		}
		buff[i] = '\0';
		LCD_GLASS_DisplayString(buff);
    }
    while(1) {
        TIM_handleTask();
        lcdIsShift = MX_LCD_Task();
        if(SP_isNewData() == true) {
            ch = SP_getData();
            if((ch == '\r') || (18u < receiveP)) {
                receiveBuff[receiveP++] = '\0';
                //receiveBuff[7] = '\0';
							LCD_GLASS_DisplayString(receiveBuff);
                (void)SP_sendString(receiveBuff);
                receiveP = 0u;
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 2000u);
            } else {
                receiveBuff[receiveP++] = ch;
            }
        }
        if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW) == true) {
            if((myRtcIsNewTime() == true) && (lcdIsShift == false)) {
                myRtcSaveActualTime();
                myRtcGetTimeString(buff);
                LCD_GLASS_DisplayString(buff);
                //LCD_GLASS_DisplayStringTime(buff);
            }
        }
        switch(CD_task()) {
            case CD_STATE_RECEIVING:    run_in_ram();   break;
            case CD_STATE_SENDING:      led2_on();      break;
            case CD_STATE_SLEEP:
                led1_off();
                led2_off();
                break;
            case CD_STATE_SENDED_CALENDAR:  LCD_GLASS_DisplayString((uint8_t*) "CALENDAR ");  break;
            case CD_STATE_SENDED_TELEPHONE: LCD_GLASS_DisplayString((uint8_t*) "TELEPHONE "); break;
            case CD_STATE_SENDED_NOTE:      LCD_GLASS_DisplayString((uint8_t*) "NOTE ");      break;
            case CD_STATE_SENDED_SCHEDULE:  LCD_GLASS_DisplayString((uint8_t*) "SCHEDULE ");  break;
            case CD_STATE_SENDED_REMINDER:  LCD_GLASS_DisplayString((uint8_t*) "REMINDER ");  break;
            case CD_STATE_SENDED_REMINDER2: LCD_GLASS_DisplayString((uint8_t*) "REMINDER ");  break;
            case CD_STATE_SENDED_FREE_FILE: LCD_GLASS_DisplayString((uint8_t*) "FREE FILE "); break;
            case CD_STATE_SENDED_DATA:
                /*
                myRtcSetTime(CD_getBuffer());
                sprintf((char*)buff, "Set time to: %s", CD_getBuffer());
                LCD_GLASS_DisplayString(buff);*/
                cdNewData = true;
                TIM_delaySetTimer(DELAY_MAIN_CASIO_NEW_DATA, 1500u);
                break;
            case CD_STATE_NOT_INIT: CD_setUserBuffer(cd_buff, sizeof(cd_buff)); break;
            default: break;
        }
        if((true == cdNewData) && (true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW))) {
            cdNewData = false;
            sprintf((char*)buff, "Set time to: %s", cd_buff);
            LCD_GLASS_DisplayString(buff);
        }

		if(tl_getTlSample().hore) {
			//led1_on();
			foo++;
			CD_sendToDiarConst(0u);
            //run_in_ram();
            
		} else {
			//led1_off();
		}
        if(tl_getTlSample().lavo) {
            myRtcGetTime((uint8_t *)buff);
            sprintf((char*)&buff[6], "-%06d", ++count);
            CD_sendToDiarConst((uint8_t *)buff);
		}
        if(tl_getTlSample().pravo) {
            //CD_sendToDiarConst(testDiar);
            (void)CD_receive();
		}
        if(tl_getTl().dole) {
			CD_senToDiarEndCommunication();
            LCD_GLASS_DisplayString(testDiar);
		} else {
		}
		if(tl_getTl().stred) {
			stredTmp = 1;
			sprintf((char*)buff, "%06d", ++count);
			LCD_GLASS_DisplayString((uint8_t*) buff);
		} else if (stredTmp) {
			if(USART2->ISR & USART_ISR_TXE) {
				stredTmp = 0;
				printf("Count: %d", count);
			}
		}

        (void)BMP180_handleTask();
        if(true == BMP180_isPresent()) {
            if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW) == true) {
                TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 4000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_PRESSURE_SHOW, 1000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                //BMP180_readTemp();
                BMP180_readPressureAndTempForced(BMP180_eOverSampleMax25_5ms);
                //sprintf((char*)buff, "%06Ld°C", sensorPresure.T);
                sprintf((char*)buff, "%d.%d°C", (int8_t)(BMP180_getTemperature() / 10u), (BMP180_getTemperature() / 100u));
                LCD_GLASS_DisplayString((uint8_t*) buff);
            }
            if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_PRESSURE_SHOW) == true) {
                TIM_delaySetTimer(DELAY_MAIN_LCD_PRESSURE_SHOW, 5000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                //sprintf((char*)buff, "%ld", BMP180_getPressure());
                sprintf((char*)buff, "%d", (int16_t)BMP180_getPressure());
                LCD_GLASS_DisplayString((uint8_t*) buff);
            }
        }
	} // while(1);
}

// kvoli testovaniu volania funkcie z RAM
__attribute__((section("RAMCODESECTION"))) void run_in_ram(void) {
  __nop();
	foo++;
	led1_on();
}

// kvoli testovaniu volania funkcie z RAM
void load_ramcode(void) {
	uint32_t i;
	char *from, *to;
	to = &Image$$RW_CODE$$Base;
	from = &Load$$RW_CODE$$Base;
	for(i = 0; i < (size_t)&Image$$RW_CODE$$Length; i++) {
		to[i] = from[i];
	}
}
