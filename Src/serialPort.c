#include "serialPort.h"
#include <stdio.h>

typedef unsigned char uchar;
#define BUFF_MAX  8U

static uchar pPlus(void);
static bool pWriteCheck(void);
static unsigned char SPbuff[BUFF_MAX];
static uchar pWrite;
static uchar pRead;
static uint8_t ucReadData;
static bool bNewData = false;

/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {
    int dummy;
};
 
/* You need this if you want use printf */
/* Struct FILE is implemented in stdio.h */
FILE __stdout;

int fputc(int ch, FILE *f) {
   GPIOB->ODR |= (1u << GPIO_ODR_OD6_Pos);
    /* Do your stuff here */
    /* SpRead your custom byte */
    /* SpRead byte to USART */
    //TM_USART_Putc(USART1, ch);
    //while(!pWriteCheck());
    SPbuff[pPlus()] = ch;
   USART2->CR1 |= USART_CR1_TXEIE;

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
		 ucReadData = USART2->RDR;
		 bNewData = true;
	 } else if((status & USART_ISR_ORE_Msk) != 0u) {
		 USART2->ICR = USART_ICR_ORECF_Msk;
	 }
   if(pRead != pWrite) {
      USART2->TDR = SPbuff[pRead++];
      if(BUFF_MAX <= pRead) {
         pRead = 0;
      }
   } else {
      USART2->CR1 &= ~USART_CR1_TXEIE;
   }
}

static bool pWriteCheck(void) {
   uchar p;
   p = pWrite;
   if((p != pRead) || (~USART2->CR1 & USART_CR1_TXEIE)) {
      return true;
   }
   return false;
}

static uchar pPlus(void) {
   while(!pWriteCheck());
   pWrite++;
   if(BUFF_MAX <= pWrite) {
      pWrite = 0;
   }
   return pWrite;
}

void SP_init(void) {
	uchar i;
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
   GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk);
   GPIOB->MODER |= (1U << GPIO_MODER_MODE6_Pos);
	 GPIOB->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED6_Pos);
   
   pWrite = BUFF_MAX;
	pRead = 0;
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
   //NVIC_EnableIRQ(USART1_IRQn);
	 for(i = 0; i < 15; i++)
			fputc((int)(i+'A'), 0);
}

bool SP_isNewData(void) {
	return bNewData;
}

uint8_t SP_getData(void) {
	bNewData = false;
	return ucReadData;
}
