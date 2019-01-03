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
#include "mfx_l4.h"
#include "tm1638_modes.h"

// SYSCLK - 4MHz
// MSI - 4MHz
// HSI16 - 16MHz
// 100: HSE clock selected - OFF
// 101: Main PLL clock selected - OFF
// 110: LSI clock selected - 32kHz
// 111: LSE clock selected - OFF

uint8_t charsToHex(uint8_t ch1, uint8_t ch2);
void charToHexString(uint8_t* buff, uint8_t ch);
void run_in_ram(void);
void load_ramcode(void);
extern char Image$$RW_CODE$$Base;        // kvoli testovaniu volania funkcie z RAM
extern char Image$$RW_CODE$$Length; // kvoli testovaniu volania funkcie z RAM
extern char Load$$RW_CODE$$Base;        // kvoli testovaniu volania funkcie z RAM
__attribute__((section("TEMPDATASECTION"), zero_init)) uint8_t foo;

uint8_t charsToHex(uint8_t ch1, uint8_t ch2) {
    uint8_t i, j, ch;
    j = 0u;
    ch = ch1;
    for(i = 0; i < 2u; i++) {
        j <<= 4u;
        if('A' <= ch) {
            ch += 9u;
        }
        j |= (ch & 0x0Fu);
        ch = ch2;
    }
    return j;
}

void charToHexString(uint8_t* buff, uint8_t ch) {
    uint8_t i;
    buff[0] = (ch & 0xF0u) >> 4u;
    for(i = 0; i < 2u; i++) {
        if(9u < buff[i]) {
            buff[i] -= 9;
            buff[i] |= 0x40u;
        } else {
            buff[i] |= 0x30u;
        }
        buff[i + 1u] = ch & 0x0Fu;
    }
}

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
    uint8_t i, j, ch;
    uint8_t buff[50];
    uint8_t cd_buff[300];
    uint32_t count = 0;
    uint8_t stredTmp = 0;

    MX_LCD_Init();
    myRtcInit();
    TIM_delayInit();
    /**** AUDIO - CS43L22 - U13 ****/
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOEEN);     // enable clock for GPIOB
    //GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);           // PB6 and PB7 to INPUT mode
    //GPIOB->OTYPER |= (uint32_t)(GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7);   // PB6 and PB7 to open-drain
    GPIOB->PUPDR |= ((1u << GPIO_PUPDR_PUPD6_Pos) & (1u << GPIO_PUPDR_PUPD7_Pos)); // Pull-up enable
    RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOBEN);  // disable clock for GPIOB

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;     // enable clock for GPIOE
    //PE2->SAI1_MCK, PE4->SAI1_FS, PE5->SAI1_SCK, PE6->SAI1_SD to log 0 via pull-down
    //PE3->AUDIO_RST to log 0
    GPIOE->MODER |= (1u << GPIO_MODER_MODE3_Pos);// PE3 to output mode (AUDIO_RST)
    GPIOE->ODR &= ~(GPIO_ODR_OD3);           // PE3 to log 0
    GPIOE->PUPDR |= ((2u << GPIO_PUPDR_PUPD2_Pos) & (2u << GPIO_PUPDR_PUPD4_Pos) & // PE2, PE4, PE5 and PE6
                     (2u << GPIO_PUPDR_PUPD5_Pos) & (2u << GPIO_PUPDR_PUPD6_Pos)); // Pull-down enable

    /**** Microphone - MP34DT01 - U17****/
    //PE9->AUDIO_CLK, PE7->AUDIO_DIN (AUDIO_DIN has HI impedance if AUDIO_CLK is UP)
    GPIOE->PUPDR |= ((1u << GPIO_PUPDR_PUPD9_Pos) & (1u << GPIO_PUPDR_PUPD7_Pos)); // Pull-up PE7 and PE9
    RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOEEN);   // disable clock for GPIOE

    SPu1_init();
    
    load_ramcode();
    tl_Init();
    LCD_GLASS_Clear();
    RCC->CFGR |= (4u << RCC_CFGR_MCOPRE_Pos); // MCO / 16 IF 4
    SP_init();

    count = 0u;
    while((false == mfx_initForced()) && (count < 100u)) {
        count++;
    }
    count = 0u;

    //LCD_GLASS_DisplayString((uint8_t *)"A");
    /*    LCD_GLASS_DisplayString(data);
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
                LCD_GLASS_DisplayString(receiveBuff);
                (void)SP_sendString(receiveBuff);
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 2000u);
                if('s' == receiveBuff[receiveP - 3u]) {
                    tm1638_sendCommand(charsToHex(receiveBuff[receiveP - 2u], receiveBuff[receiveP - 1u]));
                } else if('d' == receiveBuff[0]) {
                    tm1638_show(&receiveBuff[1]);
                } else if('a' == receiveBuff[0]) {
                    i = receiveBuff[1] & 0x0Fu;
                    tm1638_showPos(i, receiveBuff[2]);
                } else if('p' == receiveBuff[0]) {
                    i = 0u;
                    j = 1u;
                    ch = (receiveP - 1u) / 2u;
                    for(i = 0u; i < ch; i++) {
                        receiveBuff[i] = charsToHex(receiveBuff[j], receiveBuff[j + 1u]);
                        j += 2u;
                    }
                    tm1638_sendPacket(receiveBuff, ch);
                } else if('r' == receiveBuff[0]) {
                    tm1638_readTl(receiveBuff);
                    for(i = 0; i < 4u; i++) {
                        charToHexString(&receiveBuff[(i * 2u) + 4u], receiveBuff[i]);
                    }
                    (void)SP_sendBuff(&receiveBuff[4], 8u);
                }
                receiveP = 0u;
                TIM_delaySetTimer(DELAY_TM1638, 3000u);
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
            TIM_delaySetTimer(DELAY_TM1638, 3000u);
            tm1638_show("abcdEFGH");
            //run_in_ram();
            
        } else {
            //led1_off();
        }
        if(tl_getTlSample().lavo) {
            myRtcGetTime((uint8_t *)buff);
            sprintf((char*)&buff[6], "-%06d", ++count);
            CD_sendToDiarConst((uint8_t *)buff);
            mfx_iddReqMeas(10u);
        }
        if(tl_getTlSample().pravo) {
            //CD_sendToDiarConst(testDiar);
            (void)CD_receive();
            TIM_delaySetTimer(DELAY_TM1638, 3000u);
            tm1638_show("456789-.");
        }
        if(tl_getTl().dole) {
            CD_senToDiarEndCommunication();
            LCD_GLASS_DisplayString(testDiar);
            TIM_delaySetTimer(DELAY_TM1638, 3000u);
            tm1638_show("ijlnOPRS");
        } else {
        }
        if(tl_getTl().stred) {
            stredTmp = 1;
            sprintf((char*)buff, "%06d", ++count);
            LCD_GLASS_DisplayString((uint8_t*) buff);
            TIM_delaySetTimer(DELAY_TM1638, 3000u);
            tm1638_show("tuYZ0123");
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
        TMM_handleTask();
        if(MFX_STATUS_DONE == mfx_handleTask()) {
            TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 2500u);
            //sprintf((char*)buff, "%ld", (int32_t)mfx_getData());
            mfx_convertToChar(buff, mfx_getData());
            LCD_GLASS_DisplayString((uint8_t*) buff);
        }
    }
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
