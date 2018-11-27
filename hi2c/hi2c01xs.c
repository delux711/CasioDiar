/*****************************************************************************************************************
* Copyright by BSH Bosch und Siemens Hausgeraete GmbH - EDS                                                       
*                                                                                                                 
*-----------------------------------------------------------------------------------------------------------------
*                                                                                                                 
* Use, reproduction and dissemination of this software or parts of it, is not permitted without express           
* written authority of BSH EDS. The user is allowed to use this software exclusively for the development          
* of electronic boards for BSH. Usage for other purposes is strictly prohibited (e.g. development of              
* electronic boards for third parties). All rights, including copyright, rights created by patent grant           
* or registration, and rights by protection of utility patents, are reserved. Violations will be                  
* prosecuted by civil and criminal law.                                                                           
*                                                                                                                 
* The software was created and approved by acknowledged rules of technology. Because of the complexity            
* of embedded controller software the user of this software has to perform his own tests to ensure                
* proper functionality in his environment. The software was developed for usage as a software library.            
* The user is sole responsible for every other usage.  The user may modify the software for adaptations           
* needed for other microcontrollers, as well as to another compilers at own risk.                                 
* BSH will not support any adaptations or changes. BSH is authorised to use all adaptations for own purposes      
* free of charge.                                                                                                 
*                                                                                                                 
* BSH assumes no liability for the functionality or reliability of the software even in concrete                  
* applications. BSH assumes no liability for consequential damages, except in case of intention or gross          
* negligence. In any case the liability is limited to the typical and predictable damage.                         
*                                                                                                                 
* Technical changes are reserved.                                                                                 
*                                                                                                                 
******************************************************************************************************************
   PROJECT          SW_LIB_HAL
   MODULE-PREFIX    HI2C
   FILE             %PM%
   ARCHIVE          %PP%:%PI%
   PROCESSOR        ARM Cortex-M3
   COMPILER         RealView
 *****************************************************************************************************************
   AUTHOR           Simku Frantisek
   CREATED          20091016
   LAST CHANGE      %PRT%
   LAST REVISION    %PR%
   STATUS           %PS%
 *****************************************************************************************************************
   DESCRIPTION
     Configuration for I2C.
     
 *****************************************************************************************************************
   CHANGES
   %PL%

 *****************************************************************************************************************/

#include <stdio.h>
#include "hi2c.h"
#include "stm32l4xx.h"

/* TYPE DEFINITIONS ***********************************************************************************/
/* PROTOTYPES *****************************************************************************************/
/* VARIABLES ******************************************************************************************/

uint32_t HI2C_ulNewPortStatus;


/* CONSTANTS ******************************************************************************************/

/* maximum waitstate delay */
const uint8_t HI2C0_ucMaxWaitState = 0xFF;


/* DEFINITIONS ****************************************************************************************/

void HI2C0_vInitPort(void) {
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;      /* enable clock for GPIOB */
   //RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;      /* enable clock for GPIOE */
   GPIOB->OTYPER &= ~(GPIO_OTYPER_OT7);
   GPIOB->OTYPER |= (1u << GPIO_OTYPER_OT7_Pos);
   GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7);
   GPIOB->PUPDR |= (1u << GPIO_PUPDR_PUPD7_Pos); // pull-up
   
}


void HI2C0_vOutputSCL(void) {
    //HDIO_vSetModePortB6(HDIO_OUTPUT);
    GPIOB->MODER &= ~(GPIO_MODER_MODE6);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE6_Pos);
}

void HI2C0_vInputSCL(void) {
   // HDIO_vSetModePortB6(HDIO_INPUT);       
   GPIOB->MODER &= ~(GPIO_MODER_MODE6);
    GPIOB->MODER |= (0U << GPIO_MODER_MODE6_Pos);
}

void HI2C0_vClrSCL(void) {
   //HDIO_vSetPortB6(0);
   GPIOB->BSRR |= GPIO_BSRR_BR6;
}

void HI2C0_vSetSCL(void) {
   //HDIO_vSetPortB6(1);
   GPIOB->BSRR |= GPIO_BSRR_BS6;
}

bool HI2C0_bGetSCL(void) {
   //return HDIO_bGetPortB6();
   return GPIOB->IDR & GPIO_IDR_ID6;
}

void HI2C0_vOutputSDA(void) {
   //HDIO_vSetModePortB2(HDIO_OUTPUT);
   GPIOB->MODER &= ~(GPIO_MODER_MODE7);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE7_Pos);
}

void HI2C0_vInputSDA(void) {
   //HDIO_vSetModePortB2(HDIO_INPUT);
   GPIOB->MODER &= ~(GPIO_MODER_MODE7);
    GPIOB->MODER |= (0U << GPIO_MODER_MODE7_Pos);
}

void HI2C0_vClrSDA(void) {
   //HDIO_vSetPortB2(0);
   GPIOB->BSRR |= GPIO_BSRR_BR7;
}

void HI2C0_vSetSDA(void) {
   //HDIO_vSetPortB2(1);
   GPIOB->BSRR |= GPIO_BSRR_BS7;
}

bool HI2C0_bGetSDA(void) {
   //return HDIO_bGetPortB2();
   return GPIOB->IDR & GPIO_IDR_ID7;
}


void HI2C0_vBitDelayH(void)
{
   /** \todo Delay must be adjusted to get not more than 400Khz */
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
}

void HI2C0_vBitDelayL(void)
{
   /** \todo Delay must be adjusted to get not more than 400Khz */
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
}

/* for 100kHz mode this function has to wait at least 5 µsec - overhead for call/return  */
/* for 400kHz mode this function has to wait at least 1.6 µsec - overhead for call/return */
void HI2C0_vBitDly(void)
{
   /* use HSUP_vDelay(6); for 100 kHz mode */
   /* use nothing for 400 kHz mode */
}

/* event handler */
void HI2C0_vHandleEvent(void)
{

}
