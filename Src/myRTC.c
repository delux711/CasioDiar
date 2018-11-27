#include <stdint.h>
#include <stdbool.h>
#include "lcd.h"
#include "stm32l4xx.h"                  // Device header

void myRtcUpdate(void);

typedef unsigned char uchar;
#define ON  ((bool) 1)
#define OFF ((bool) 0)

typedef struct DIGIT_s {
   uchar value;
   bool  update;
} digit_t;
typedef struct CLOCK_STRING_s {
   digit_t hodDes;
   digit_t hodJed;
   digit_t minDes;
   digit_t minJed;
   digit_t sekDes;
   digit_t sekJed;
   uchar rtcString[6];
} CLOCK_STRING_t;

CLOCK_STRING_t rtcStr;

void myRtcInit(void) {
   unsigned char timeout = 0;
   RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN_Msk;
   PWR->CR1 |= PWR_CR1_DBP;

   RTC->WPR = 0xCA;
   RTC->WPR = 0x53;
	 RCC->BDCR |= RCC_BDCR_RTCEN;
   RTC->CR &= ~RTC_CR_WUTE;
	 RTC->CR &= ~RTC_CR_WUCKSEL;
   RTC->CR |= (0U << RTC_CR_WUCKSEL_Pos);
   RTC->WUTR = 2047;
   RTC->CR |= RTC_CR_WUTE;

   // initialization TIME and DATE
   RTC->ISR |= RTC_ISR_INIT;
   do {
		 timeout--;
   } while(timeout && !(RTC->ISR & RTC_ISR_INITF));
   RTC->TR = RTC->BKP0R;  // TIME
   // RTC->DR = 0;        // DATE
   RTC->ISR &= ~RTC_ISR_INIT;

   PWR->CR1 &= ~PWR_CR1_DBP;
   //RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN_Msk;
   myRtcUpdate();
}

void myRtcLcd(void) {
   uchar str[6];
   uint8_t i, ch;
   if(RTC->ISR & RTC_ISR_WUTF) {
      RTC->ISR &= ~RTC_ISR_WUTF;
      str[0] = (uchar)(((RTC->TR & RTC_TR_HT)  >> RTC_TR_HT_Pos)  + '0');
      str[1] = (uchar)(((RTC->TR & RTC_TR_HU)  >> RTC_TR_HU_Pos)  + '0');
      str[2] = (uchar)(((RTC->TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos) + '0');
      str[3] = (uchar)(((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos) + '0');
      str[4] = (uchar)(((RTC->TR & RTC_TR_ST)  >> RTC_TR_ST_Pos)  + '0');
      str[5] = (uchar)(((RTC->TR & RTC_TR_SU)  >> RTC_TR_SU_Pos)  + '0');
      i = 0;
      do {
        ch = str[i];
        if(ch == '0')
          str[i++] = ' ';
        else break;
      } while(i < 6);
      PWR->CR1 |= PWR_CR1_DBP;
      RTC->BKP0R = RTC->TR;
      PWR->CR1 &= ~PWR_CR1_DBP;
      LCD_GLASS_DisplayStringTime(str);
   }
   /*
   if(rtcStr.hodDes.update == ON) {
      LCD_GLASS_WriteChar(&rtcStr.hodDes.value, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_1);
      rtcStr.hodDes.update = OFF;
   }

   if(rtcStr.sekJed.update == ON) {
      LCD_GLASS_WriteChar(&rtcStr.sekJed.value, POINT_OFF, DOUBLEPOINT_OFF, LCD_DIGIT_POSITION_6);
      rtcStr.sekJed.update = OFF;
   }*/
   
}

void myRtcUpdate(void) {
   /*
   rtcStr.hodDes.value = ((RTC->TR & RTC_TR_HT) >> RTC_TR_HT_Pos) + "0";
   rtcStr.hodDes.update = 1;
   rtcStr.hodJed.value = ((RTC->TR & RTC_TR_HU) >> RTC_TR_HU_Pos) + "0";
   rtcStr.hodJed.update = 1;
   rtcStr.minDes.value = ((RTC->TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos) + "0";
   rtcStr.minDes.update = 1;
   rtcStr.minJed.value = ((RTC->TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos) + "0";
   rtcStr.minJed.update = 1;
   rtcStr.sekDes.value = ((RTC->TR & RTC_TR_ST) >> RTC_TR_ST_Pos) + "0";
   rtcStr.sekDes.update = 1;
   rtcStr.sekJed.value = ((RTC->TR & RTC_TR_SU) >> RTC_TR_SU_Pos) + "0";
   rtcStr.sekJed.update = 1;*/
}
