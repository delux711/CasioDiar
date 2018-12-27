/**
  ******************************************************************************
  * File Name          : LCD.c
  * Description        : This file provides code for the configuration
  *                      of the LCD instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lcd.h"
#include "timerLib.h"

/* USER CODE BEGIN 0 */

#define ASCII_CHAR_0                  (0x30u)  /* 0 */
#define ASCII_CHAR_AT_SYMBOL          (0x40u)  /* @ */
#define ASCII_CHAR_LEFT_OPEN_BRACKET  (0x5Bu)  /* [ */
#define ASCII_CHAR_APOSTROPHE         (0x60u)  /* ` */
#define ASCII_CHAR_LEFT_OPEN_BRACE    (0x7Bu)  /* ( */

/**
  @verbatim
================================================================================
                              GLASS LCD MAPPING
================================================================================
LCD allows to display informations on six 14-segment digits and 4 bars:

  1       2       3       4       5       6
-----   -----   -----   -----   -----   -----   
|\|/| o |\|/| o |\|/| o |\|/| o |\|/|   |\|/|   BAR3
-- --   -- --   -- --   -- --   -- --   -- --   BAR2
|/|\| o |/|\| o |/|\| o |/|\| o |/|\|   |/|\|   BAR1
----- * ----- * ----- * ----- * -----   -----   BAR0

LCD segment mapping:
--------------------
  -----A-----        _ 
  |\   |   /|   COL |_|
  F H  J  K B          
  |  \ | /  |        _ 
  --G-- --M--   COL |_|
  |  / | \  |          
  E Q  P  N C          
  |/   |   \|        _ 
  -----D-----   DP  |_|

 An LCD character coding is based on the following matrix:
COM           0   1   2     3
SEG(n)      { E , D , P ,   N   }
SEG(n+1)    { M , C , COL , DP  }
SEG(23-n-1) { B , A , K ,   J   }
SEG(23-n)   { G , F , Q ,   H   }
with n positive odd number.

 The character 'A' for example is:
  -------------------------------
LSB   { 1 , 0 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
      { 1 , 1 , 0 , 0   }
MSB   { 1 , 1 , 0 , 0   }
      -------------------
  'A' =  F    E   0   0 hexa

  @endverbatim
*/

/* Constant table for cap characters 'A' --> 'Z' */
const uint16_t CapLetterMap[26]= {
    /* A        B        C        D        E        F        G        H        I  */
    0xFE00u, 0x6714u, 0x1D00u, 0x4714u, 0x9D00u, 0x9C00u, 0x3F00u, 0xFA00u, 0x0014u,
    /* J        K        L        M        N        O        P        Q        R  */
    0x5300u, 0x9841u, 0x1900u, 0x5A48u, 0x5A09u, 0x5F00u, 0xFC00u, 0x5F01u, 0xFC01u,
    /* S        T        U        V        W        X        Y        Z  */
    0xAF00u, 0x0414u, 0x5b00u, 0x18C0u, 0x5A81u, 0x00C9u, 0x0058u, 0x05C0u
};
/* Constant table for cap characters 'a' --> 'z' */
const uint16_t CapLetterMapLowercase[26]= {
    /* a        b        c        d        e        f        g        h        i  */
    0x2380u, 0x9901u, 0xB100u, 0x6380u, 0x9180u, 0x9C00u, 0x6340u, 0xBA00u, 0x0010u,
    /* j        k        l        m        n        o        p        q        r  */
    0x4300u, 0x0055u, 0x0014u, 0xB210u, 0x2210u, 0xB300u, 0x9C40u, 0xEC01u, 0x2010u,
    /* s        t        u        v        w        x        y        z  */
    0x2101u, 0x9900u, 0x1300u, 0x1080u, 0x1281u, 0x00C9u, 0x6304u, 0x8180u
};

/* Constant table for number '0' --> '9' */
const uint16_t NumberMap[10]=
    {
        /* 0      1      2      3      4      5      6      7      8      9  */
        0x5F00,0x4240,0xF500,0x6700,0xEa00,0xAF00,0xBF00,0x04600,0xFF00,0xEF00
    };

uint32_t Digit[4];     /* Digit frame buffer */
static uint8_t mx_lcd_buff[MX_LCD_MAX_BUFFER];
static uint8_t mx_lcd_buffP;
static uint8_t mx_lcd_buffSize;
static bool mx_lcd_isShift = false;

static void Convert(uint8_t* Char, Point_Typedef Point, DoublePoint_Typedef Colon);
static void LCD_GLASS_show_part(void);

/* USER CODE END 0 */

/* LCD init function */
void MX_LCD_Init(void)
{
    unsigned char timeout = 0;
    uint32_t backUp;
    RCC->APB1ENR1 |= RCC_APB1ENR1_LCDEN_Msk | RCC_APB1ENR1_PWREN_Msk;
    PWR->CR1 |= PWR_CR1_DBP;

    // back up of RTC backup register
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;
    backUp = RTC->BKP0R;

     // BDCR reset
    RCC->BDCR |= RCC_BDCR_BDRST;
    do {
        timeout--;
    } while(timeout && (~RCC->BDCR & RCC_BDCR_BDRST));   // while for reset BDCR register
    RCC->BDCR &= ~RCC_BDCR_BDRST;
    RTC->BKP0R = backUp;

    // start LSE
    RCC->BDCR |= RCC_BDCR_LSEON;
    timeout = 0;
    do {
        timeout--;
    } while(timeout && (~RCC->BDCR & RCC_BDCR_LSERDY));  // while for LSE ready

    // Check LSE oscillator
    if(RCC->BDCR & RCC_BDCR_LSERDY) {
        RCC->BDCR |= (1u << RCC_BDCR_RTCSEL_Pos); // 01: LSE oscillator clock used as RTC clock
    } else {
        // if LSE not work
        RCC->BDCR &= ~RCC_BDCR_LSEON;             // LSE works not correct. Switch to LSI
        RCC->CSR |= RCC_CSR_LSION;
        timeout = 0;
        do {
            timeout--;
        } while(timeout && (~RCC->CSR & RCC_CSR_LSIRDY));
    RCC->BDCR |= (2u << RCC_BDCR_RTCSEL_Pos); // 10: LSI oscillator clock used as RTC clock
    }
    RCC->BDCR |= RCC_BDCR_RTCEN;
    PWR->CR1 &= ~PWR_CR1_DBP;
    RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN_Msk;

    /**LCD GPIO Configuration    
    PA6     -------> LCD_SEG3        COM0  PA8          SEG23 PA6
    PA7     -------> LCD_SEG4        COM1  PA9          SEG0  PA7
    PA8     -------> LCD_COM0        COM2  PA10         COM0  PA8
    PA9     -------> LCD_COM1        COM3  PB9          COM1  PA9
    PA10     ------> LCD_COM2        SEG0  PA7          COM2  PA10
    PA15     ------> LCD_SEG17       SEG1  PC5          SEG10 PA15
                                     SEG2  PB1          
    PB0     -------> LCD_SEG5        SEG3  PB13         SEG21 PB0
    PB1     -------> LCD_SEG6        SEG4  PB15         SEG2  PB1
    PB4     -------> LCD_SEG8        SEG5  PD9          SEG11 PB4
    PB5     -------> LCD_SEG9        SEG6  PD11         SEG12 PB5
    PB9     -------> LCD_COM3        SEG7  PD13         COM3  PB9
    PB12     ------> LCD_SEG12       SEG8  PD15         SEG20 PB12
    PB13     ------> LCD_SEG13       SEG9  PC7          SEG3  PB13
    PB14     ------> LCD_SEG14       SEG10 PA15         SEG19 PB14
    PB15     ------> LCD_SEG15       SEG11 PB4          SEG4  PB15
                                     SEG12 PB5          
    PC3     -------> LCD_VLCD        SEG13 PC8          VLCD  PC3
    PC4     -------> LCD_SEG22       SEG14 PC6          SEG22 PC4
    PC5     -------> LCD_SEG23       SEG15 PD14         SEG1  PC5
    PC6     -------> LCD_SEG24       SEG16 PD12         SEG14 PC6
    PC7     -------> LCD_SEG25       SEG17 PD10         SEG9  PC7
    PC8     -------> LCD_SEG26       SEG18 PD8          SEG13 PC8
                                     SEG19 PB14         
    PD8     -------> LCD_SEG28       SEG20 PB12         SEG18 PD8
    PD9     -------> LCD_SEG29       SEG21 PB0          SEG5  PD9
    PD10     ------> LCD_SEG30       SEG22 PC4          SEG17 PD10
    PD11     ------> LCD_SEG31       SEG23 PA6          SEG6  PD11
    PD12     ------> LCD_SEG32       VLCD  PC3          SEG16 PD12
    PD13     ------> LCD_SEG33                          SEG7  PD13
    PD14     ------> LCD_SEG34                          SEG15 PD14
    PD15     ------> LCD_SEG35                          SEG8  PD15
    */

    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN);
    GPIOA->MODER &= (~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE15));
    GPIOA->MODER |= ((2U << GPIO_MODER_MODE6_Pos) | (2U << GPIO_MODER_MODE7_Pos) | (2U << GPIO_MODER_MODE8_Pos) |
                     (2U << GPIO_MODER_MODE9_Pos) | (2U << GPIO_MODER_MODE10_Pos)| (2U << GPIO_MODER_MODE15_Pos));
    GPIOB->MODER &= (~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE9 |
                       GPIO_MODER_MODE12 | GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15));
    GPIOB->MODER |= ((2U << GPIO_MODER_MODE0_Pos) | (2U << GPIO_MODER_MODE1_Pos) | ( 2U << GPIO_MODER_MODE4_Pos) |
                     (2U << GPIO_MODER_MODE5_Pos) | (2U << GPIO_MODER_MODE9_Pos) | (2U << GPIO_MODER_MODE12_Pos) |
                     (2U << GPIO_MODER_MODE13_Pos)| (2U << GPIO_MODER_MODE14_Pos)| (2U << GPIO_MODER_MODE15_Pos));
    GPIOC->MODER &= (~(GPIO_MODER_MODE3 | GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7 | GPIO_MODER_MODE8));
    GPIOC->MODER |= ((2U << GPIO_MODER_MODE3_Pos) | (2U << GPIO_MODER_MODE4_Pos) | (2U << GPIO_MODER_MODE5_Pos) |
                     (2U << GPIO_MODER_MODE6_Pos) | (2U << GPIO_MODER_MODE7_Pos) | (2U << GPIO_MODER_MODE8_Pos));
    GPIOD->MODER &= (~(GPIO_MODER_MODE8 | GPIO_MODER_MODE9 | GPIO_MODER_MODE10 | GPIO_MODER_MODE11 |
                       GPIO_MODER_MODE12| GPIO_MODER_MODE13| GPIO_MODER_MODE14 | GPIO_MODER_MODE15));
    GPIOD->MODER |= ((2U << GPIO_MODER_MODE8_Pos) | (2U << GPIO_MODER_MODE9_Pos) | ( 2U << GPIO_MODER_MODE10_Pos) |
                     (2U << GPIO_MODER_MODE11_Pos) | (2U << GPIO_MODER_MODE12_Pos) |
                     (2U << GPIO_MODER_MODE13_Pos) | (2U << GPIO_MODER_MODE14_Pos) | (2U << GPIO_MODER_MODE15_Pos));
    //AFR AF11    
    GPIOA->AFR[0] |= ((11U << GPIO_AFRL_AFSEL6_Pos) | (11U << GPIO_AFRL_AFSEL7_Pos));
    GPIOA->AFR[1] |= ((11U << GPIO_AFRH_AFSEL8_Pos) | (11U << GPIO_AFRH_AFSEL9_Pos) |
                      (11U << GPIO_AFRH_AFSEL10_Pos)| (11U << GPIO_AFRH_AFSEL15_Pos));
    GPIOB->AFR[0] |= ((11U << GPIO_AFRL_AFSEL0_Pos) | (11U << GPIO_AFRL_AFSEL1_Pos) |
                      (11U << GPIO_AFRL_AFSEL4_Pos) | (11U << GPIO_AFRL_AFSEL5_Pos));
    GPIOB->AFR[1] |= ((11U << GPIO_AFRH_AFSEL9_Pos) | (11U << GPIO_AFRH_AFSEL12_Pos) | (11U << GPIO_AFRH_AFSEL13_Pos) |
                      (11U << GPIO_AFRH_AFSEL14_Pos)| (11U << GPIO_AFRH_AFSEL15_Pos));
    GPIOC->AFR[0] |= ((11U << GPIO_AFRL_AFSEL3_Pos) | (11U << GPIO_AFRL_AFSEL4_Pos) | (11U << GPIO_AFRL_AFSEL5_Pos) |
                      (11U << GPIO_AFRL_AFSEL6_Pos) | (11U << GPIO_AFRL_AFSEL7_Pos));
    GPIOC->AFR[1] |=  (11U << GPIO_AFRH_AFSEL8_Pos);
    GPIOD->AFR[1] |= ((11U << GPIO_AFRH_AFSEL8_Pos) | (11U << GPIO_AFRH_AFSEL9_Pos)  | (11U << GPIO_AFRH_AFSEL10_Pos) |
                      (11U << GPIO_AFRH_AFSEL11_Pos)| (11U << GPIO_AFRH_AFSEL12_Pos) | (11U << GPIO_AFRH_AFSEL13_Pos) |
                      (11U << GPIO_AFRH_AFSEL14_Pos)| (11U << GPIO_AFRH_AFSEL15_Pos));

    LCD->FCR = 0x3C1450;
    LCD->CR = 0x4C;
    LCD->CR |= 1;
    /*
    hlcd.Instance = LCD;
    hlcd.Init.Prescaler = LCD_PRESCALER_1;
    hlcd.Init.Divider = LCD_DIVIDER_31;
    hlcd.Init.Duty = LCD_DUTY_1_4;
    hlcd.Init.Bias = LCD_BIAS_1_3;
    hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
    hlcd.Init.Contrast = LCD_CONTRASTLEVEL_5;
    hlcd.Init.DeadTime = LCD_DEADTIME_0;
    hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_5;
    hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
    hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
    hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
    hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;
    HAL_LCD_Init(&hlcd);
    */
}

/* USER CODE BEGIN 1 */

bool MX_LCD_Task(void) {
    if((TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW) == true) && (mx_lcd_isShift == true)) {
        TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 500u);
        LCD_GLASS_show_part();
    }
    return mx_lcd_isShift;
}

void LCD_GLASS_show_part(void) {
    uint8_t ch, pointer;
    Point_Typedef point;
    DoublePoint_Typedef colon;
    DigitPosition_Typedef position = LCD_DIGIT_POSITION_1;

    if((mx_lcd_buff[mx_lcd_buffP] == '.') || (mx_lcd_buff[mx_lcd_buffP] == ':')) {
        mx_lcd_buffP++;
    }
    pointer = mx_lcd_buffP;

    while((LCD->SR & LCD_SR_UDR) != 0u);
    while(position <= LCD_DIGIT_POSITION_6) {
        if(pointer < mx_lcd_buffSize) {
            ch = mx_lcd_buff[pointer++];
        } else {
            ch = ' ';
        }
        if(mx_lcd_buff[pointer] == '.') {
            point = POINT_ON;
            pointer++;
        } else {
            point = POINT_OFF;
        }
        if(mx_lcd_buff[pointer] == ':') {
            colon = DOUBLEPOINT_ON;
            pointer++;
        } else {
            colon = DOUBLEPOINT_OFF;
        }
        /* Write one character on LCD */
        LCD_GLASS_WriteChar(&ch, point, colon, position);
        position++;
    }
    mx_lcd_buffP++;
    if(mx_lcd_buffP >= (mx_lcd_buffSize - 1u)) {
        mx_lcd_isShift = false;
    }
    /* Update the LCD display */
    LCD->SR |= LCD_SR_UDR;
}

/**
  * @brief  Write a character string in the LCD RAM buffer.
  * @param  ptr: Pointer to string to display on the LCD Glass.
  * @retval None
  */
void LCD_GLASS_DisplayString(uint8_t* ptr) {
    uint8_t ch, size;

    size = 0u;
    mx_lcd_buffP = 0u;
    mx_lcd_buffSize = 0u;
    mx_lcd_isShift = false;
    do {
        ch = ptr[mx_lcd_buffSize];
        mx_lcd_buff[mx_lcd_buffSize++] = ch;
        if((ch == '.') || (ch == ':'))  {
            size++;
        }
    } while((ch != '\0') && (mx_lcd_buffSize < MX_LCD_MAX_BUFFER));
    mx_lcd_buff[MX_LCD_MAX_BUFFER - 1] = '\0';
    mx_lcd_buffSize--;
    if((mx_lcd_buffSize - size) > 6u) {
        mx_lcd_isShift = true;
        TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 500u);
    }
    LCD_GLASS_show_part();
}

void LCD_GLASS_DisplayStringTime(uint8_t* ptr) {
    DigitPosition_Typedef position = LCD_DIGIT_POSITION_1;

    /* Send the string character by character on lCD */
    while ((*ptr != '\0') && (position <= LCD_DIGIT_POSITION_6)) {
        /* Write one character on LCD */
        LCD_GLASS_WriteChar(ptr, POINT_OFF, (DoublePoint_Typedef)(position & 0x01), position);

        /* Point on the next character */
        ptr++;

        /* Increment the character counter */
        position++;
    }
    /* Update the LCD display */
    LCD->SR |= LCD_SR_UDR;
}

/**
  * @brief  Clear the whole LCD RAM buffer.
  * @retval None
  */
void LCD_GLASS_Clear(void) {
    unsigned char i;
    for(i=0; i<16; i++) {
        LCD->RAM[i] = 0;
    }
    /* Update the LCD display */
    LCD->CLR |= LCD_CLR_UDDC;
}

/**
  * @brief  Write a character in the LCD frame buffer.
  * @param  ch: the character to display.
  * @param  Point: a point to add in front of char
  *         This parameter can be: POINT_OFF or POINT_ON
  * @param  Colon: flag indicating if a colon character has to be added in front
  *         of displayed character.
  *         This parameter can be: DOUBLEPOINT_OFF or DOUBLEPOINT_ON.           
  * @param  Position: position in the LCD of the character to write [1:6]
  * @retval None
  */
void LCD_GLASS_WriteChar(uint8_t* ch, Point_Typedef Point, DoublePoint_Typedef Colon, DigitPosition_Typedef Position) {
    uint32_t data =0x00;
    /* To convert displayed character in segment in array digit */
    Convert(ch, (Point_Typedef)Point, (DoublePoint_Typedef)Colon);

    switch (Position) {
    /* Position 1 on LCD (Digit1)*/
    case LCD_DIGIT_POSITION_1:
        data = ((Digit[0] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG1_SHIFT)
                | (((Digit[0] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG23_SHIFT);
        MODIFY_REG(LCD->RAM[0], ~LCD_DIGIT1_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */


        data = ((Digit[1] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG1_SHIFT)
                | (((Digit[1] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG23_SHIFT);
        MODIFY_REG(LCD->RAM[2], ~LCD_DIGIT1_COM1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = ((Digit[2] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG1_SHIFT)
                | (((Digit[2] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG23_SHIFT);
        MODIFY_REG(LCD->RAM[4], ~LCD_DIGIT1_COM2_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = ((Digit[3] & 0x1) << LCD_SEG0_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG1_SHIFT)
                | (((Digit[3] & 0x4) >> 2) << LCD_SEG22_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG23_SHIFT);
        MODIFY_REG(LCD->RAM[6], ~LCD_DIGIT1_COM3_SEG_MASK, data); /* 1H 1J 1DP 1N */
        break;

    // /* Position 2 on LCD (Digit2)*/
    case LCD_DIGIT_POSITION_2:
        data = ((Digit[0] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG3_SHIFT)
                | (((Digit[0] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG21_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT2_COM0], ~LCD_DIGIT2_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = ((Digit[1] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG3_SHIFT)
                | (((Digit[1] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG21_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT2_COM1], ~LCD_DIGIT2_COM1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = ((Digit[2] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG3_SHIFT)
                | (((Digit[2] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG21_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT2_COM2], ~LCD_DIGIT2_COM2_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = ((Digit[3] & 0x1) << LCD_SEG2_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG3_SHIFT)
                | (((Digit[3] & 0x4) >> 2) << LCD_SEG20_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG21_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT2_COM3], ~LCD_DIGIT2_COM3_SEG_MASK, data); /* 1H 1J 1DP 1N */
        break;

    // /* Position 3 on LCD (Digit3)*/
    case LCD_DIGIT_POSITION_3:
        data = ((Digit[0] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG5_SHIFT)
                | (((Digit[0] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG19_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT3_COM0], ~LCD_DIGIT3_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = ((Digit[1] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG5_SHIFT)
                | (((Digit[1] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG19_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT3_COM1], ~LCD_DIGIT3_COM1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = ((Digit[2] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG5_SHIFT)
                | (((Digit[2] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG19_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT3_COM2], ~LCD_DIGIT3_COM2_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = ((Digit[3] & 0x1) << LCD_SEG4_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG5_SHIFT)
                | (((Digit[3] & 0x4) >> 2) << LCD_SEG18_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG19_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT3_COM3], ~LCD_DIGIT3_COM3_SEG_MASK, data); /* 1H 1J 1DP 1N */
        break;

    // /* Position 4 on LCD (Digit4)*/
    case LCD_DIGIT_POSITION_4:
        data = ((Digit[0] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG17_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM0], ~LCD_DIGIT4_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = (((Digit[0] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[0] & 0x4) >> 2) << LCD_SEG16_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM0_1], ~LCD_DIGIT4_COM0_1_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = ((Digit[1] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG17_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM1], ~LCD_DIGIT4_COM1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = (((Digit[1] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[1] & 0x4) >> 2) << LCD_SEG16_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM1_1], ~LCD_DIGIT4_COM1_1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = ((Digit[2] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG17_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM2], ~LCD_DIGIT4_COM2_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = (((Digit[2] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[2] & 0x4) >> 2) << LCD_SEG16_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM2_1], ~LCD_DIGIT4_COM2_1_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = ((Digit[3] & 0x1) << LCD_SEG6_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG17_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM3], ~LCD_DIGIT4_COM3_SEG_MASK, data); /* 1H 1J 1DP 1N */

        data = (((Digit[3] & 0x2) >> 1) << LCD_SEG7_SHIFT) | (((Digit[3] & 0x4) >> 2) << LCD_SEG16_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT4_COM3_1], ~LCD_DIGIT4_COM3_1_SEG_MASK, data); /* 1H 1J 1DP 1N */
        break;

    // /* Position 5 on LCD (Digit5)*/
    case LCD_DIGIT_POSITION_5:
        data = (((Digit[0] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[0] & 0x4) >> 2) << LCD_SEG14_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM0], ~LCD_DIGIT5_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = ((Digit[0] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG15_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM0_1], ~LCD_DIGIT5_COM0_1_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = (((Digit[1] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[1] & 0x4) >> 2) << LCD_SEG14_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM1], ~LCD_DIGIT5_COM1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = ((Digit[1] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG15_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM1_1], ~LCD_DIGIT5_COM1_1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = (((Digit[2] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[2] & 0x4) >> 2) << LCD_SEG14_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM2], ~LCD_DIGIT5_COM2_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = ((Digit[2] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG15_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM2_1], ~LCD_DIGIT5_COM2_1_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = (((Digit[3] & 0x2) >> 1) << LCD_SEG9_SHIFT) | (((Digit[3] & 0x4) >> 2) << LCD_SEG14_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM3], ~LCD_DIGIT5_COM3_SEG_MASK, data); /* 1H 1J 1DP 1N */

        data = ((Digit[3] & 0x1) << LCD_SEG8_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG15_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT5_COM3_1], ~LCD_DIGIT5_COM3_1_SEG_MASK, data); /* 1H 1J 1DP 1N */
        break;

    // /* Position 6 on LCD (Digit6)*/
    case LCD_DIGIT_POSITION_6:
        data = ((Digit[0] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[0] & 0x2) >> 1) << LCD_SEG11_SHIFT)
                | (((Digit[0] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[0] & 0x8) >> 3) << LCD_SEG13_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT6_COM0], ~LCD_DIGIT6_COM0_SEG_MASK, data); /* 1G 1B 1M 1E */

        data = ((Digit[1] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[1] & 0x2) >> 1) << LCD_SEG11_SHIFT)
                | (((Digit[1] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[1] & 0x8) >> 3) << LCD_SEG13_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT6_COM1], ~LCD_DIGIT6_COM1_SEG_MASK, data); /* 1F 1A 1C 1D */

        data = ((Digit[2] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[2] & 0x2) >> 1) << LCD_SEG11_SHIFT)
                | (((Digit[2] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[2] & 0x8) >> 3) << LCD_SEG13_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT6_COM2], ~LCD_DIGIT6_COM2_SEG_MASK, data); /* 1Q 1K 1Col 1P */

        data = ((Digit[3] & 0x1) << LCD_SEG10_SHIFT) | (((Digit[3] & 0x2) >> 1) << LCD_SEG11_SHIFT)
                | (((Digit[3] & 0x4) >> 2) << LCD_SEG12_SHIFT) | (((Digit[3] & 0x8) >> 3) << LCD_SEG13_SHIFT);
        MODIFY_REG(LCD->RAM[LCD_DIGIT6_COM3], ~LCD_DIGIT6_COM3_SEG_MASK, data); /* 1H 1J 1DP 1N */
        break;

    default:
        break;
    }
}

/**
  * @brief  Convert an ascii char to the a LCD digit.
  * @param  Char: a char to display.
  * @param  Point: a point to add in front of char
  *         This parameter can be: POINT_OFF or POINT_ON
  * @param  Colon : flag indicating if a colon character has to be added in front
  *         of displayed character.
  *         This parameter can be: DOUBLEPOINT_OFF or DOUBLEPOINT_ON.
  * @retval None
  */
static void Convert(uint8_t* Char, Point_Typedef Point, DoublePoint_Typedef Colon) {
    uint16_t ch = 0 ;
    uint8_t loop = 0, index = 0;
  
    switch (*Char) {
        case ' ':  ch = 0x00; break;
        case '*':  ch = C_STAR; break;
        case '(':  ch = C_OPENPARMAP; break;
        case ')':  ch = C_CLOSEPARMAP; break;
        case 'µ': ch = C_UMAP; break;
        case '-':  ch = C_MINUS; break;
        case '+':  ch = C_PLUS; break;
        case '/':  ch = C_SLATCH; break;  
        case '°': ch = C_PERCENT_1; break;  
        case '%':  ch = C_PERCENT_2; break;
        case ',':  ch = C_COMMA; break;
        case 255:  ch = C_FULL; break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9': ch = NumberMap[*Char - ASCII_CHAR_0]; break;
        default:
        /* The character Char is one letter in upper case*/
        if ( (*Char < ASCII_CHAR_LEFT_OPEN_BRACKET) && (*Char > ASCII_CHAR_AT_SYMBOL)) {
            ch = CapLetterMap[*Char - 'A'];
        }
        /* The character Char is one letter in lower case*/
        if((*Char < ASCII_CHAR_LEFT_OPEN_BRACE) && ( *Char > ASCII_CHAR_APOSTROPHE)) {
            ch = CapLetterMapLowercase[*Char - 'a'];
        }
        break;
    }

    /* Set the digital point can be displayed if the point is on */
    if(Point == POINT_ON) {
        ch |= 0x0002;
    }

    /* Set the "COL" segment in the character that can be displayed if the colon is on */
    if(Colon == DOUBLEPOINT_ON) {
        ch |= 0x0020;
    }    

    for(loop = 12,index=0 ;index < 4; loop -= 4,index++) {
        Digit[index] = (ch >> loop) & 0x0f; /*To isolate the less significant digit */
    }
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

// HAL_StatusTypeDef HAL_LCD_Write(LCD_HandleTypeDef *hlcd, uint32_t RAMRegisterIndex, uint32_t RAMRegisterMask, uint32_t Data)
// {
  // uint32_t tickstart = 0x00;

  // if((hlcd->State == HAL_LCD_STATE_READY) || (hlcd->State == HAL_LCD_STATE_BUSY))
  // {
    // /* Check the parameters */
    // assert_param(IS_LCD_RAM_REGISTER(RAMRegisterIndex));

    // if(hlcd->State == HAL_LCD_STATE_READY)
    // {
      // /* Process Locked */
      // __HAL_LOCK(hlcd);
      // hlcd->State = HAL_LCD_STATE_BUSY;

      // /* Get timeout */
      // tickstart = HAL_GetTick();

      // /*!< Wait Until the LCD is ready */
      // while(__HAL_LCD_GET_FLAG(hlcd, LCD_FLAG_UDR) != RESET)
      // {
        // if((HAL_GetTick() - tickstart ) > LCD_TIMEOUT_VALUE)
        // {
          // hlcd->ErrorCode = HAL_LCD_ERROR_UDR;

          // /* Process Unlocked */
          // __HAL_UNLOCK(hlcd);

          // return HAL_TIMEOUT;
        // }
      // }
    // }

    // /* Copy the new Data bytes to LCD RAM register */
    // MODIFY_REG(hlcd->Instance->RAM[RAMRegisterIndex], ~(RAMRegisterMask), Data);

    // return HAL_OK;
  // }
  // else
  // {
    // return HAL_ERROR;
  // }
// }

// /**
  // * @brief Clear the LCD RAM registers.
  // * @param hlcd: LCD handle
  // * @retval None
  // */
// HAL_StatusTypeDef HAL_LCD_Clear(LCD_HandleTypeDef *hlcd)
// {
  // uint32_t tickstart = 0x00;
  // uint32_t counter = 0;

  // if((hlcd->State == HAL_LCD_STATE_READY) || (hlcd->State == HAL_LCD_STATE_BUSY))
  // {
    // /* Process Locked */
    // __HAL_LOCK(hlcd);

    // hlcd->State = HAL_LCD_STATE_BUSY;

    // /* Get timeout */
    // tickstart = HAL_GetTick();

    // /*!< Wait Until the LCD is ready */
    // while(__HAL_LCD_GET_FLAG(hlcd, LCD_FLAG_UDR) != RESET)
    // {
      // if((HAL_GetTick() - tickstart ) > LCD_TIMEOUT_VALUE)
      // {
        // hlcd->ErrorCode = HAL_LCD_ERROR_UDR;

        // /* Process Unlocked */
        // __HAL_UNLOCK(hlcd);

        // return HAL_TIMEOUT;
      // }
    // }
    // /* Clear the LCD_RAM registers */
    // for(counter = LCD_RAM_REGISTER0; counter <= LCD_RAM_REGISTER15; counter++)
    // {
      // hlcd->Instance->RAM[counter] = 0;
    // }

    // /* Update the LCD display */
    // HAL_LCD_UpdateDisplayRequest(hlcd);

    // return HAL_OK;
  // }
  // else
  // {
    // return HAL_ERROR;
  // }
// }
