#include "mfx_l4_i2c.h"
#include "stm32l4xx.h"

/* maximum waitstate delay */
const uint8_t HI2Cmfx_ucMaxWaitState = 0xFF;


static uint8_t HI2Cmfx_ucError;
static uint8_t HI2Cmfx_ucLastRx;
static bool HI2Cmfx_bEventEnabled;

/* FUNCTIONS */
static void HI2Cmfx_vWaitForSlave(void);
static void HI2Cmfx_vMakeStopCondition(void);


/*************************************************************************************/

void HI2Cmfx_vInitPort(void) {
   RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;      /* enable clock for GPIOB */
   //RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;      /* enable clock for GPIOB */
   GPIOB->OTYPER &= ~(GPIO_OTYPER_OT11);
   GPIOB->OTYPER |= (1u << GPIO_OTYPER_OT11_Pos);
   GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD11);
   GPIOB->PUPDR |= (1u << GPIO_PUPDR_PUPD11_Pos); // pull-up   
}


void HI2Cmfx_vOutputSCL(void) {
    //HDIO_vSetModePortB10(HDIO_OUTPUT);
    GPIOB->MODER &= ~(GPIO_MODER_MODE10);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE10_Pos);
}

void HI2Cmfx_vInputSCL(void) {
   // HDIO_vSetModePortB10(HDIO_INPUT);       
   GPIOB->MODER &= ~(GPIO_MODER_MODE10);
    GPIOB->MODER |= (0U << GPIO_MODER_MODE10_Pos);
}

void HI2Cmfx_vClrSCL(void) {
   //HDIO_vSetPortB10(0);
   GPIOB->BSRR |= GPIO_BSRR_BR10;
}

void HI2Cmfx_vSetSCL(void) {
   //HDIO_vSetPortB10(1);
   GPIOB->BSRR |= GPIO_BSRR_BS10;
}

bool HI2Cmfx_bGetSCL(void) {
   //return HDIO_bGetPortB10();
   return GPIOB->IDR & GPIO_IDR_ID10;
}

void HI2Cmfx_vOutputSDA(void) {
   //HDIO_vSetModePortB2(HDIO_OUTPUT);
   GPIOB->MODER &= ~(GPIO_MODER_MODE11);
    GPIOB->MODER |= (1U << GPIO_MODER_MODE11_Pos);
}

void HI2Cmfx_vInputSDA(void) {
   //HDIO_vSetModePortB2(HDIO_INPUT);
   GPIOB->MODER &= ~(GPIO_MODER_MODE11);
    GPIOB->MODER |= (0U << GPIO_MODER_MODE11_Pos);
}

void HI2Cmfx_vClrSDA(void) {
   //HDIO_vSetPortB2(0);
   GPIOB->BSRR |= GPIO_BSRR_BR11;
}

void HI2Cmfx_vSetSDA(void) {
   //HDIO_vSetPortB2(1);
   GPIOB->BSRR |= GPIO_BSRR_BS11;
}

bool HI2Cmfx_bGetSDA(void) {
   //return HDIO_bGetPortB2();
   return GPIOB->IDR & GPIO_IDR_ID11;
}


void HI2Cmfx_vBitDelayH(void)
{
   /** \todo Delay must be adjusted to get not more than 400Khz */
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
   __asm("NOP"); 
}

void HI2Cmfx_vBitDelayL(void)
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
/* for 400kHz mode this function has to wait at least 1.10 µsec - overhead for call/return */
void HI2Cmfx_vBitDly(void)
{
   /* use HSUP_vDelay(10); for 100 kHz mode */
   /* use nothing for 400 kHz mode */
}

/* event handler */
void HI2Cmfx_vHandleEvent(void)
{

}


/*************************************************************************************/

/* HI2Cmfx_vWaitForSlave() */
/* set SCL to a high state and wait until the slave releases it too */
static void HI2Cmfx_vWaitForSlave(void) {
   uint8_t ucWait = HI2Cmfx_ucMaxWaitState;

   HI2Cmfx_vInputSCL();
   HI2Cmfx_vSetSCL();

   while(HI2Cmfx_bGetSCL() == 0u) {
      ucWait--;
      if(ucWait == 0u) {
         /* SCL stuck? ... */
         HI2Cmfx_ucError |= HI2C_TIMEOUT;
         return;
      }
   }
   HI2Cmfx_vOutputSCL();
   HI2Cmfx_vBitDelayH();
}

/* HI2Cmfx_vMakeStopCondition() */
static void HI2Cmfx_vMakeStopCondition(void) {
   /* generate stop condition */
   HI2Cmfx_vOutputSDA();
   HI2Cmfx_vClrSDA();
   HI2Cmfx_vBitDelayL();
   HI2Cmfx_vWaitForSlave();
   HI2Cmfx_vSetSDA();
   HI2Cmfx_vBitDelayH();
   /* release bus */
   HI2Cmfx_vInputSCL();
   HI2Cmfx_vInputSDA();

}

void HI2Cmfx_vInit(uint8_t ucIndex) {
#ifdef HI2C_SPECIAL_INIT 
   HI2Cmfx_vInitPort(); /* Initialize the special features of the periphery. */
#endif
   /* preset for port latch */
   HI2Cmfx_vInputSCL();
   HI2Cmfx_vInputSDA();
}

/* HI2Cmfx_bSetAddr() */
bool HI2Cmfx_bSetAddr(uint8_t ucAddress) {
   /* no errors so far */
   HI2Cmfx_ucError = 0u;

   HI2Cmfx_vSetSDA();
   HI2Cmfx_vSetSCL();

   /* generate start condition */
   HI2Cmfx_vOutputSDA();
   HI2Cmfx_vOutputSCL();
   HI2Cmfx_vBitDelayH();
   HI2Cmfx_vClrSDA()   ;
   HI2Cmfx_vBitDelayH();
   HI2Cmfx_vClrSCL()   ;
   HI2Cmfx_vBitDelayL();

   /* transmit address */
   return HI2Cmfx_bSetTxData(ucAddress, 0u);
}

/* HI2Cmfx_bSetTxData() */
bool HI2Cmfx_bSetTxData(uint8_t ucDataByte, bool bStop) {
   uint8_t ucCounter;

   /* transmit all 8 data bits */
   ucCounter = 8u;
   do {
      /* send each bit, MSB first */
      if((ucDataByte & 0x80u) != 0u) {
         HI2Cmfx_vSetSDA();
      }
      else {
         HI2Cmfx_vClrSDA();
      }
      ucDataByte = (uint8_t)(ucDataByte << 1u);

      /* generate clock */
      HI2Cmfx_vSetSCL();
      HI2Cmfx_vBitDelayH();
      HI2Cmfx_vClrSCL();
      HI2Cmfx_vBitDelayL();
      --ucCounter;
   } while(ucCounter > 0u);

   /* listen for ACK */
   HI2Cmfx_vSetSDA();
   HI2Cmfx_vInputSDA();
   HI2Cmfx_vWaitForSlave();

   if(HI2Cmfx_bGetSDA() != 0u) {
      /* ack didn't happen, may be nothing out there */
      HI2Cmfx_ucError |= HI2C_NACK;
   }
   HI2Cmfx_vClrSCL();
   HI2Cmfx_vBitDelayL();
   HI2Cmfx_vSetSDA();
   HI2Cmfx_vOutputSDA();

   if((bStop != 0u) || (HI2Cmfx_ucError != 0u)) {
      /* generate stop condition */
      HI2Cmfx_vMakeStopCondition();
   }
   /* call event handler */

   if(HI2Cmfx_bEventEnabled != 0u) {
      HI2Cmfx_vHandleEvent();
   }
   return (bool)(HI2Cmfx_ucError == 0u);
}

/* HI2Cmfx_vTriggerReceive() */
uint8_t HI2Cmfx_vTriggerReceive(bool bStop) {
   uint8_t ucCounter;

   /* switch to input since we want to receive data */
   HI2Cmfx_vInputSDA();

   /* receive the bits -- starting with the MSB */
   ucCounter = 8u;

   do {
      HI2Cmfx_vSetSCL();
      HI2Cmfx_vBitDelayH();

      HI2Cmfx_ucLastRx <<= 1u;

      if(HI2Cmfx_bGetSDA() != 0u) {
         HI2Cmfx_ucLastRx |= 1u;
      }
      HI2Cmfx_vClrSCL();
      HI2Cmfx_vBitDelayL();
      --ucCounter;
   } while(ucCounter > 0u);

   /* send ACK according to the stop flag */
   HI2Cmfx_vOutputSDA();

   if(bStop != 0u) {
      /* no acknowledge */
      HI2Cmfx_vSetSDA();
   } else {
      /* acknowledge */
      HI2Cmfx_vClrSDA();
   }
   HI2Cmfx_vWaitForSlave();
   HI2Cmfx_vClrSCL();
   HI2Cmfx_vSetSDA();
   HI2Cmfx_vBitDelayL();

   if(bStop != 0u) {
      HI2Cmfx_vMakeStopCondition();
   }
   /* call event handler */

   if(HI2Cmfx_bEventEnabled != 0u) {
      HI2Cmfx_vHandleEvent();
   }
   return HI2Cmfx_ucLastRx;
}

void HI2Cmfx_vSendStop(void) {
   /* Send stop-condition.  Changing of SDA during SCL = OUTPUT_HIGH is */
   /* only allowed while sending start- or stop-condition.  For generating */
   /* stop-condition both SDA and SCL must be LOW */
   HI2Cmfx_vClrSCL();
   HI2Cmfx_vClrSDA();
   HI2Cmfx_vOutputSCL();
   HI2Cmfx_vOutputSDA();

   HI2Cmfx_vBitDelayL();           /* clock low period */
   HI2Cmfx_vSetSCL();              /* change SCL edge to OUTPUT_HIGH level */
   HI2Cmfx_vBitDelayH();           /* Stop condition setup time */
   HI2Cmfx_vSetSDA();;             /* change SDA edge to OUTPUT_HIGH level */

   HI2Cmfx_vInputSCL();
   HI2Cmfx_vInputSDA();
}

bool HI2Cmfx_bForceBusRelease(void) {
   uint16_t uiCounter;
   bool bSdaStatus;

   HI2Cmfx_vOutputSCL();
   HI2Cmfx_vInputSDA();

   for(uiCounter = 0u; uiCounter < 15u; uiCounter++) {
      HI2Cmfx_vClrSCL();          /* security: send some clocks to free the bus-lines */
      HI2Cmfx_vBitDelayL();          /* clock low period */
      HI2Cmfx_vSetSCL();
      HI2Cmfx_vBitDelayH();
   }
   HI2Cmfx_vSendStop();

   /* check BUS-lines */
   for(uiCounter = 0u; uiCounter < 5u; uiCounter++) {
      bSdaStatus = HI2Cmfx_bGetSDA();
      if(bSdaStatus != 0u) {
         break;
      }
   }
   return (bSdaStatus);
}
