#include "getJoystick.h"

static TL_ENU out;

TL_ENU tl_getTlSample(void) {
    out.extTl = !(GPIOB->ODR & GPIO_ODR_OD0_Msk);

    out.lavo = 0;
    out.pravo = 0;
    out.hore = 0;
    out.dole = 0;
    out.stred = 0;

    if(GPIOA->IDR & GPIO_IDR_ID0) {
        out.stred = 1;
    }
    if(GPIOA->IDR & GPIO_IDR_ID1) {
        out.lavo = 1;
    }
    if(GPIOA->IDR & GPIO_IDR_ID5) {
        out.dole = 1;
    }
    if(GPIOA->IDR & GPIO_IDR_ID2) {
        out.pravo = 1;
    }
    if(GPIOA->IDR & GPIO_IDR_ID3) {
        out.hore = 1;
    }
    return out;
}

void tl_Init(void) {
    RCC->AHB2ENR |= (1U << 2);                    /* Enable GPIOC clock         */
    
//    SET_BIT(RCC->AHB2ENR, RCC_AHB2ENR_GPIOAEN_Msk);
    
    RCC->AHB2ENR |= ((1U << RCC_AHB2ENR_GPIOAEN_Pos) | (1U << RCC_AHB2ENR_GPIOBEN_Pos) | (1U << RCC_AHB2ENR_GPIOEEN_Pos));
    GPIOA->MODER &= 0xABFFF300; // PA0, PA1, PA2, PA3, PA5
    GPIOA->PUPDR |= ((2U << GPIO_PUPDR_PUPD0_Pos) | (2U << GPIO_PUPDR_PUPD1_Pos) | (2U << GPIO_PUPDR_PUPD2_Pos) |
        (2U << GPIO_PUPDR_PUPD3_Pos) | (2U << GPIO_PUPDR_PUPD5_Pos));
    GPIOB->MODER &= ~(GPIO_MODER_MODE2_Msk);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE2_Pos);    // led4
    GPIOE->MODER &= (~GPIO_MODER_MODE8_Msk);
    GPIOE->MODER |= (1U << GPIO_MODER_MODE8_Pos);
    HDIO_testPinInit();
/*
    GPIOC->MODER   &= ~(3U << 2* 13);
    GPIOC->OSPEEDR &= ~(3U << 2* 13);
    GPIOC->OSPEEDR |=  (1U << 2* 13);
    GPIOC->PUPDR   &= ~(3U << 2* 13);*/
}
/*
typedef enum _TL_ENU {
    lavo,
    pravo,
    hore,
    dole,
    stred,
    extTl
} TL_ENU;
*/

TL_ENU tl_getTl(void) {
    return out;
}    
