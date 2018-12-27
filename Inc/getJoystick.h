#include "stm32l4xx.h"                  // Device header

typedef unsigned int uint;
#define led1_0()      do { GPIOB->ODR &= ~GPIO_ODR_OD2_Msk; } while(0)
#define led1_1()      do { GPIOB->ODR |=  GPIO_ODR_OD2_Msk; } while(0)
#define led1_on()     led1_1()
#define led1_off()    led1_0()
#define led2_0()      do { GPIOE->ODR &= ~GPIO_ODR_OD8_Msk; } while(0)
#define led2_1()      do { GPIOE->ODR |=  GPIO_ODR_OD8_Msk; } while(0)
#define led2_on()     led2_1()
#define led2_off()    led2_0()

#define HDIO_testPinOn()     do { GPIOD->ODR |= GPIO_ODR_OD0_Msk; }    while(0) // pin ON
#define HDIO_testPinOff()    do { GPIOD->ODR &= (~GPIO_ODR_OD0_Msk); } while(0) // pin OFF
#define HDIO_testPinInit()   do { RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN_Msk; \
                                 GPIOD->MODER &= (~GPIO_MODER_MODE0_Msk); \
                                 GPIOD->MODER |= (1u << GPIO_MODER_MODE0_Pos); } while(0)

typedef struct _TL_ENU {
    uint lavo :1;
    uint pravo:1;
    uint hore :1;
    uint dole :1;
    uint stred:1;
    uint extTl:1;
} TL_ENU;

extern TL_ENU tl_getTlSample(void);
extern TL_ENU tl_getTl(void);
extern void tl_Init(void);
