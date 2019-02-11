#include "hi2c0.h"
#include "stm32l4xx.h"

/* maximum waitstate delay */
const uint8_t HI2C0_ucMaxWaitState = 0xFF;

static uint8_t ucChipAddr;
static uint8_t HI2C0_ucError;
static uint8_t HI2C0_ucLastRx;
static bool HI2C0_bEventEnabled;
static bool bIsChippresent = false;

/* FUNCTIONS */
static void HI2C0_vWaitForSlave(void);
static void HI2C0_vMakeStopCondition(void);


/*************************************************************************************/

void HI2C0_vInitPort(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;      /* enable clock for GPIOx */
    GPIOE->OTYPER &= ~(GPIO_OTYPER_OT15);
    GPIOE->OTYPER |= (1u << GPIO_OTYPER_OT15_Pos);
    GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD15);
    GPIOE->PUPDR |= (1u << GPIO_PUPDR_PUPD15_Pos); // pull-up   
}


void HI2C0_vOutputSCL(void) {
    GPIOE->MODER &= ~(GPIO_MODER_MODE14);
    GPIOE->MODER |= (1U << GPIO_MODER_MODE14_Pos);
}

void HI2C0_vInputSCL(void) {
    GPIOE->MODER &= ~(GPIO_MODER_MODE14);
    GPIOE->MODER |= (0U << GPIO_MODER_MODE14_Pos);
}

void HI2C0_vClrSCL(void) {
    GPIOE->BSRR |= GPIO_BSRR_BR14;
}

void HI2C0_vSetSCL(void) {
    GPIOE->BSRR |= GPIO_BSRR_BS14;
}

bool HI2C0_bGetSCL(void) {
    return GPIOE->IDR & GPIO_IDR_ID14;
}

void HI2C0_vOutputSDA(void) {
    GPIOE->MODER &= ~(GPIO_MODER_MODE15);
    GPIOE->MODER |= (1U << GPIO_MODER_MODE15_Pos);
}

void HI2C0_vInputSDA(void) {
    GPIOE->MODER &= ~(GPIO_MODER_MODE15);
    GPIOE->MODER |= (0U << GPIO_MODER_MODE15_Pos);
}

void HI2C0_vClrSDA(void) {
   GPIOE->BSRR |= GPIO_BSRR_BR15;
}

void HI2C0_vSetSDA(void) {
   GPIOE->BSRR |= GPIO_BSRR_BS15;
}

bool HI2C0_bGetSDA(void) {
    return GPIOE->IDR & GPIO_IDR_ID15;
}


void HI2C0_vBitDelayH(void) {
    /** \todo Delay must be adjusted to get not more than 400Khz */
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
}

void HI2C0_vBitDelayL(void) {
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
void HI2C0_vBitDly(void) {
    /* use HSUP_vDelay(10); for 100 kHz mode */
    /* use nothing for 400 kHz mode */
}

/* event handler */
void HI2C0_vHandleEvent(void) {

}

/*************************************************************************************/

/* set SCL to a high state and wait until the slave releases it too */
static void HI2C0_vWaitForSlave(void) {
    uint8_t ucWait = HI2C0_ucMaxWaitState;

    HI2C0_vInputSCL();
    HI2C0_vSetSCL();

    while(HI2C0_bGetSCL() == 0u) {
        ucWait--;
        if(ucWait == 0u) {
            /* SCL stuck? ... */
            HI2C0_ucError |= HI2C0_TIMEOUT;
            return;
        }
    }
    HI2C0_vOutputSCL();
    HI2C0_vBitDelayH();
}

static void HI2C0_vMakeStopCondition(void) {
    /* generate stop condition */
    HI2C0_vOutputSDA();
    HI2C0_vClrSDA();
    HI2C0_vBitDelayL();
    HI2C0_vWaitForSlave();
    HI2C0_vSetSDA();
    HI2C0_vBitDelayH();
    /* release bus */
    HI2C0_vInputSCL();
    HI2C0_vInputSDA();
}

void HI2C0_vInit(uint8_t chipAddress) {
    HI2C0_setChipAddress(chipAddress);
#ifdef HI2C_SPECIAL_INIT 
    HI2C0_vInitPort(); /* Initialize the special features of the periphery. */
#endif
    /* preset for port latch */
    HI2C0_vInputSCL();
    HI2C0_vInputSDA();
}

bool HI2C0_bSetAddr(uint8_t ucAddress) {
    /* no errors so far */
    HI2C0_ucError = 0u;

    HI2C0_vSetSDA();
    HI2C0_vSetSCL();

    /* generate start condition */
    HI2C0_vOutputSDA();
    HI2C0_vOutputSCL();
    HI2C0_vBitDelayH();
    HI2C0_vClrSDA()   ;
    HI2C0_vBitDelayH();
    HI2C0_vClrSCL()   ;
    HI2C0_vBitDelayL();

    /* transmit address */
    return HI2C0_bSetTxData(ucAddress, 0u);
}

/* HI2C0_bSetTxData() */
bool HI2C0_bSetTxData(uint8_t ucDataByte, bool bStop) {
    uint8_t ucCounter;

    /* transmit all 8 data bits */
    ucCounter = 8u;
    do {
        /* send each bit, MSB first */
        if((ucDataByte & 0x80u) != 0u) {
            HI2C0_vSetSDA();
        }
        else {
            HI2C0_vClrSDA();
        }
        ucDataByte = (uint8_t)(ucDataByte << 1u);

        /* generate clock */
        HI2C0_vSetSCL();
        HI2C0_vBitDelayH();
        HI2C0_vClrSCL();
        HI2C0_vBitDelayL();
        --ucCounter;
   } while(ucCounter > 0u);

    /* listen for ACK */
    HI2C0_vSetSDA();
    HI2C0_vInputSDA();
    HI2C0_vWaitForSlave();

    if(HI2C0_bGetSDA() != 0u) {
        /* ack didn't happen, may be nothing out there */
        HI2C0_ucError |= HI2C0_NACK;
    }
    HI2C0_vClrSCL();
    HI2C0_vBitDelayL();
    HI2C0_vSetSDA();
    HI2C0_vOutputSDA();

    if((bStop != 0u) || (HI2C0_ucError != 0u)) {
        /* generate stop condition */
        HI2C0_vMakeStopCondition();
    }
    /* call event handler */

    if(HI2C0_bEventEnabled != 0u) {
        HI2C0_vHandleEvent();
    }
    return (bool)(HI2C0_ucError == 0u);
}

/* HI2C0_vTriggerReceive() */
uint8_t HI2C0_vTriggerReceive(bool bStop) {
    uint8_t ucCounter;

    /* switch to input since we want to receive data */
    HI2C0_vInputSDA();

    /* receive the bits -- starting with the MSB */
    ucCounter = 8u;

    do {
        HI2C0_vSetSCL();
        HI2C0_vBitDelayH();

        HI2C0_ucLastRx <<= 1u;

        if(HI2C0_bGetSDA() != 0u) {
            HI2C0_ucLastRx |= 1u;
        }
        HI2C0_vClrSCL();
        HI2C0_vBitDelayL();
        --ucCounter;
   } while(ucCounter > 0u);

    /* send ACK according to the stop flag */
    HI2C0_vOutputSDA();

    if(bStop != 0u) {
        /* no acknowledge */
        HI2C0_vSetSDA();
    } else {
        /* acknowledge */
        HI2C0_vClrSDA();
    }
    HI2C0_vWaitForSlave();
    HI2C0_vClrSCL();
    HI2C0_vSetSDA();
    HI2C0_vBitDelayL();

    if(bStop != 0u) {
        HI2C0_vMakeStopCondition();
    }
    /* call event handler */
    if(HI2C0_bEventEnabled != 0u) {
        HI2C0_vHandleEvent();
    }
    return HI2C0_ucLastRx;
}

void HI2C0_vSendStop(void) {
    /* Send stop-condition.  Changing of SDA during SCL = OUTPUT_HIGH is */
    /* only allowed while sending start- or stop-condition.  For generating */
    /* stop-condition both SDA and SCL must be LOW */
    HI2C0_vClrSCL();
    HI2C0_vClrSDA();
    HI2C0_vOutputSCL();
    HI2C0_vOutputSDA();

    HI2C0_vBitDelayL();           /* clock low period */
    HI2C0_vSetSCL();              /* change SCL edge to OUTPUT_HIGH level */
    HI2C0_vBitDelayH();           /* Stop condition setup time */
    HI2C0_vSetSDA();;             /* change SDA edge to OUTPUT_HIGH level */

    HI2C0_vInputSCL();
    HI2C0_vInputSDA();
}

bool HI2C0_bForceBusRelease(void) {
    uint16_t uiCounter;
    bool bSdaStatus;

    HI2C0_vOutputSCL();
    HI2C0_vInputSDA();

    for(uiCounter = 0u; uiCounter < 15u; uiCounter++) {
        HI2C0_vClrSCL();          /* security: send some clocks to free the bus-lines */
        HI2C0_vBitDelayL();          /* clock low period */
        HI2C0_vSetSCL();
        HI2C0_vBitDelayH();
    }
    HI2C0_vSendStop();

    /* check BUS-lines */
    for(uiCounter = 0u; uiCounter < 5u; uiCounter++) {
        bSdaStatus = HI2C0_bGetSDA();
        if(bSdaStatus != 0u) {
            break;
        }
    }
    return (bSdaStatus);
}

uint8_t HI2C0_readByteForced(uint8_t addr, bool stop) {
    uint8_t ret;
    ret = 0u;
    if(true == HI2C0_writeAddrForced(addr, true)) {
        if(true == HI2C0_bSetAddr(ucChipAddr | 0x01u)) { // read
            ret = HI2C0_vTriggerReceive(stop);
        }
    }
    return ret;
}
    
bool HI2C0_writeByteForced(uint8_t addr, bool stop, uint8_t data) {
    bool ret;
    ret = false;
    if(true == HI2C0_writeAddrForced(addr, false)) { // write
        if(true == HI2C0_bSetTxData(data, stop)) { // write address
            ret = true;
        }
    }
    return ret;
}

bool HI2C0_writeAddrForced(uint8_t addr, bool stop) {
    bool ret;
    ret = false;
    if(true == HI2C0_bSetAddr(ucChipAddr)) { // write
        bIsChippresent = true;
        if(true == HI2C0_bSetTxData(addr, stop)) { // write address
            ret = true;
        } else {
            bIsChippresent = false;
        }
    } else {
        bIsChippresent = false;
    }
    return ret;
}

void HI2C0_setChipAddress(uint8_t chipAddress) {
    ucChipAddr = (0xFEu & chipAddress);
}

uint8_t HI2C0_getChipAddress(void) {
    return ucChipAddr;
}

bool HI2C0_isChipPresent(void) {
    return bIsChippresent;
}
