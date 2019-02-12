#include "serialPort.h"
#include <stdio.h>

typedef unsigned char uint8_t;
#define BUFF_MAX  50U


static uint8_t SP_pPlus(void);
static bool SP_pWriteCheck(void);
static unsigned char SP_buff[BUFF_MAX];
static uint8_t SP_pWrite;
static uint8_t SP_pRead;
static uint8_t SP_ucReadData;
static bool SP_bNewData = false;
static bool SP_bPause = false;

/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {
    int dummy;
};
 
/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
FILE __stdout;

int fputc(int ch, FILE *f) {
    /* Do your stuff here */
    /* SpRead your custom byte */
    /* SpRead byte to USART */
    SP_buff[SP_pPlus()] = ch;
    USART2->CR1 |= USART_CR1_TXEIE;

    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}

uint8_t SP_sendBuff(uint8_t *buff, uint8_t length) {
    uint8_t i;
    for(i = 0; i < length; i++) {
        SP_sendChar(buff[i]);
    }
      return 1;
}

uint8_t SP_sendString(uint8_t *buff) {
    uint8_t i = 0;
    uint8_t ch = buff[0];
    while(ch != '\0') {
        SP_sendChar(ch);
        ch = buff[++i];
    }
      return 1;
}

uint8_t SP_sendChar(uint8_t ch) {
    //GPIOB->ODR |= (1u << GPIO_ODR_OD6_Pos);
    /* Do your stuff here */
    /* SSP_pRead your custom byte */
    /* SSP_pRead byte to USART */
    //TM_USART_Putc(USART2, ch);
    //while(!u1_n());
    SP_buff[SP_pPlus()] = ch;
    if(SP_bPause == false) {
        USART2->CR1 |= USART_CR1_TXEIE;
    }

    /* If everything is OK, you have to return character written */
    //GPIOB->ODR &= ~(1u << GPIO_ODR_OD6_Pos);
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}

void USART2_IRQHandler(void) {
    uint32_t status;
    status = USART2->ISR;  /* Read the status register to clear the flags. */

    if((status & USART_ISR_RXNE) != 0u) {
        SP_ucReadData = USART2->RDR;
        SP_bNewData = true;
    } else if((status & USART_ISR_ORE_Msk) != 0u) {
        USART2->ICR = USART_ICR_ORECF_Msk;
    }
    if(SP_pRead != SP_pWrite) {
        USART2->TDR = SP_buff[SP_pRead++];
        if(BUFF_MAX <= SP_pRead) {
            SP_pRead = 0;
        }
    } else {
        USART2->CR1 &= ~USART_CR1_TXEIE;
    }
}

void SP_pauseOn(void) {
    USART2->CR1 &= ~USART_CR1_TXEIE;
    SP_bPause = true;
}

void SP_pauseOff(void) {
    SP_bPause = false;
    USART2->CR1 |= USART_CR1_TXEIE;
}

bool SP_pWriteCheck(void) {
    if((SP_pWrite != SP_pRead) || (~USART2->CR1 & USART_CR1_TXEIE)) {
        return true;
    }
    return false;
}

uint8_t SP_pPlus(void) {
    uint8_t temp;
    while(!SP_pWriteCheck());
    temp = SP_pWrite;
    SP_pWrite++;
    if(BUFF_MAX <= SP_pWrite) {
        SP_pWrite = 0;
    }
    return temp;
}

void SP_init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE6_Pos);
    GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED6_Pos);

    SP_pWrite = BUFF_MAX;
    SP_pRead = 0;
    NVIC_EnableIRQ(USART2_IRQn);
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
    RCC->CR |= RCC_CR_HSION;
    while(!RCC->CR & RCC_CR_HSIRDY);
    RCC->CCIPR |= (2U << RCC_CCIPR_USART2SEL_Pos); // 10: HSI16 clock selected as USART2 clock
    //USART_TX -> PD5
    //USART_RX -> PD6
    GPIOD->MODER &= (~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6));
    GPIOD->MODER |= ((2U << GPIO_MODER_MODE5_Pos) | (2U << GPIO_MODER_MODE6_Pos));
    GPIOD->AFR[0] |= ((7U << GPIO_AFRL_AFSEL5_Pos) | (7U << GPIO_AFRL_AFSEL6_Pos));
    USART2->BRR = (16 * 1000000) / (96 * 100); 
    /* Enable the USART unit. */
    //USART2->CR1 = USART_CR1_UE;
    /* Set TE and RE bits */
    USART2->CR1 |= USART_CR1_UE;
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    USART2->CR1 |= USART_CR1_RXNEIE;
    /* Set the global interrupt into enabled state. */
    //NVIC_EnableIRQ(USART2_IRQn);
    // fputc((int)'\r', &__stdout);
    // fputc((int)'\n', &__stdout);
}

bool SP_isNewData(void) {
    return SP_bNewData;
}

uint8_t SP_getData(void) {
    SP_bNewData = false;
    return SP_ucReadData;
}
