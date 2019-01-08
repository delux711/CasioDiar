#include "hi2cxxCHANNELxx.h"
#include "stm32l4xx.h"

/* maximum waitstate delay */
const uint8_t HI2CxxCHANNELxx_ucMaxWaitState = 0xFF;

static uint8_t ucChipAddr;
static uint8_t HI2CxxCHANNELxx_ucError;
static uint8_t HI2CxxCHANNELxx_ucLastRx;
static bool HI2CxxCHANNELxx_bEventEnabled;
static bool bIsChippresent = false;

/* FUNCTIONS */
static void HI2CxxCHANNELxx_vWaitForSlave(void);
static void HI2CxxCHANNELxx_vMakeStopCondition(void);


/*************************************************************************************/

void HI2CxxCHANNELxx_vInitPort(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;      /* enable clock for GPIOx */
    GPIOxxSDAPORTxx->OTYPER &= ~(GPIO_OTYPER_OTxxSDAPINxx);
    GPIOxxSDAPORTxx->OTYPER |= (1u << GPIO_OTYPER_OTxxSDAPINxx_Pos);
    GPIOxxSDAPORTxx->PUPDR &= ~(GPIO_PUPDR_PUPDxxSDAPINxx);
    GPIOxxSDAPORTxx->PUPDR |= (1u << GPIO_PUPDR_PUPDxxSDAPINxx_Pos); // pull-up   
}


void HI2CxxCHANNELxx_vOutputSCL(void) {
    GPIOxxCLKPORTxx->MODER &= ~(GPIO_MODER_MODExxCLKPINxx);
    GPIOxxCLKPORTxx->MODER |= (1U << GPIO_MODER_MODExxCLKPINxx_Pos);
}

void HI2CxxCHANNELxx_vInputSCL(void) {
    GPIOxxCLKPORTxx->MODER &= ~(GPIO_MODER_MODExxCLKPINxx);
    GPIOxxCLKPORTxx->MODER |= (0U << GPIO_MODER_MODExxCLKPINxx_Pos);
}

void HI2CxxCHANNELxx_vClrSCL(void) {
    GPIOxxCLKPORTxx->BSRR |= GPIO_BSRR_BRxxCLKPINxx;
}

void HI2CxxCHANNELxx_vSetSCL(void) {
    GPIOxxCLKPORTxx->BSRR |= GPIO_BSRR_BSxxCLKPINxx;
}

bool HI2CxxCHANNELxx_bGetSCL(void) {
    return GPIOxxCLKPORTxx->IDR & GPIO_IDR_IDxxCLKPINxx;
}

void HI2CxxCHANNELxx_vOutputSDA(void) {
    GPIOxxSDAPORTxx->MODER &= ~(GPIO_MODER_MODExxSDAPINxx);
    GPIOxxSDAPORTxx->MODER |= (1U << GPIO_MODER_MODExxSDAPINxx_Pos);
}

void HI2CxxCHANNELxx_vInputSDA(void) {
    GPIOxxSDAPORTxx->MODER &= ~(GPIO_MODER_MODExxSDAPINxx);
    GPIOxxSDAPORTxx->MODER |= (0U << GPIO_MODER_MODExxSDAPINxx_Pos);
}

void HI2CxxCHANNELxx_vClrSDA(void) {
   GPIOxxSDAPORTxx->BSRR |= GPIO_BSRR_BRxxSDAPINxx;
}

void HI2CxxCHANNELxx_vSetSDA(void) {
   GPIOxxSDAPORTxx->BSRR |= GPIO_BSRR_BSxxSDAPINxx;
}

bool HI2CxxCHANNELxx_bGetSDA(void) {
    return GPIOxxSDAPORTxx->IDR & GPIO_IDR_IDxxSDAPINxx;
}


void HI2CxxCHANNELxx_vBitDelayH(void) {
    /** \todo Delay must be adjusted to get not more than 400Khz */
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
    __asm("NOP"); 
}

void HI2CxxCHANNELxx_vBitDelayL(void) {
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
void HI2CxxCHANNELxx_vBitDly(void) {
    /* use HSUP_vDelay(10); for 100 kHz mode */
    /* use nothing for 400 kHz mode */
}

/* event handler */
void HI2CxxCHANNELxx_vHandleEvent(void) {

}

/*************************************************************************************/

/* set SCL to a high state and wait until the slave releases it too */
static void HI2CxxCHANNELxx_vWaitForSlave(void) {
    uint8_t ucWait = HI2CxxCHANNELxx_ucMaxWaitState;

    HI2CxxCHANNELxx_vInputSCL();
    HI2CxxCHANNELxx_vSetSCL();

    while(HI2CxxCHANNELxx_bGetSCL() == 0u) {
        ucWait--;
        if(ucWait == 0u) {
            /* SCL stuck? ... */
            HI2CxxCHANNELxx_ucError |= HI2CxxCHANNELxx_TIMEOUT;
            return;
        }
    }
    HI2CxxCHANNELxx_vOutputSCL();
    HI2CxxCHANNELxx_vBitDelayH();
}

static void HI2CxxCHANNELxx_vMakeStopCondition(void) {
    /* generate stop condition */
    HI2CxxCHANNELxx_vOutputSDA();
    HI2CxxCHANNELxx_vClrSDA();
    HI2CxxCHANNELxx_vBitDelayL();
    HI2CxxCHANNELxx_vWaitForSlave();
    HI2CxxCHANNELxx_vSetSDA();
    HI2CxxCHANNELxx_vBitDelayH();
    /* release bus */
    HI2CxxCHANNELxx_vInputSCL();
    HI2CxxCHANNELxx_vInputSDA();
}

void HI2CxxCHANNELxx_vInit(uint8_t chipAddress) {
    HI2CxxCHANNELxx_setChipAddress(chipAddress);
#ifdef HI2C_SPECIAL_INIT 
    HI2CxxCHANNELxx_vInitPort(); /* Initialize the special features of the periphery. */
#endif
    /* preset for port latch */
    HI2CxxCHANNELxx_vInputSCL();
    HI2CxxCHANNELxx_vInputSDA();
}

bool HI2CxxCHANNELxx_bSetAddr(uint8_t ucAddress) {
    /* no errors so far */
    HI2CxxCHANNELxx_ucError = 0u;

    HI2CxxCHANNELxx_vSetSDA();
    HI2CxxCHANNELxx_vSetSCL();

    /* generate start condition */
    HI2CxxCHANNELxx_vOutputSDA();
    HI2CxxCHANNELxx_vOutputSCL();
    HI2CxxCHANNELxx_vBitDelayH();
    HI2CxxCHANNELxx_vClrSDA()   ;
    HI2CxxCHANNELxx_vBitDelayH();
    HI2CxxCHANNELxx_vClrSCL()   ;
    HI2CxxCHANNELxx_vBitDelayL();

    /* transmit address */
    return HI2CxxCHANNELxx_bSetTxData(ucAddress, 0u);
}

/* HI2CxxCHANNELxx_bSetTxData() */
bool HI2CxxCHANNELxx_bSetTxData(uint8_t ucDataByte, bool bStop) {
    uint8_t ucCounter;

    /* transmit all 8 data bits */
    ucCounter = 8u;
    do {
        /* send each bit, MSB first */
        if((ucDataByte & 0x80u) != 0u) {
            HI2CxxCHANNELxx_vSetSDA();
        }
        else {
            HI2CxxCHANNELxx_vClrSDA();
        }
        ucDataByte = (uint8_t)(ucDataByte << 1u);

        /* generate clock */
        HI2CxxCHANNELxx_vSetSCL();
        HI2CxxCHANNELxx_vBitDelayH();
        HI2CxxCHANNELxx_vClrSCL();
        HI2CxxCHANNELxx_vBitDelayL();
        --ucCounter;
   } while(ucCounter > 0u);

    /* listen for ACK */
    HI2CxxCHANNELxx_vSetSDA();
    HI2CxxCHANNELxx_vInputSDA();
    HI2CxxCHANNELxx_vWaitForSlave();

    if(HI2CxxCHANNELxx_bGetSDA() != 0u) {
        /* ack didn't happen, may be nothing out there */
        HI2CxxCHANNELxx_ucError |= HI2CxxCHANNELxx_NACK;
    }
    HI2CxxCHANNELxx_vClrSCL();
    HI2CxxCHANNELxx_vBitDelayL();
    HI2CxxCHANNELxx_vSetSDA();
    HI2CxxCHANNELxx_vOutputSDA();

    if((bStop != 0u) || (HI2CxxCHANNELxx_ucError != 0u)) {
        /* generate stop condition */
        HI2CxxCHANNELxx_vMakeStopCondition();
    }
    /* call event handler */

    if(HI2CxxCHANNELxx_bEventEnabled != 0u) {
        HI2CxxCHANNELxx_vHandleEvent();
    }
    return (bool)(HI2CxxCHANNELxx_ucError == 0u);
}

/* HI2CxxCHANNELxx_vTriggerReceive() */
uint8_t HI2CxxCHANNELxx_vTriggerReceive(bool bStop) {
    uint8_t ucCounter;

    /* switch to input since we want to receive data */
    HI2CxxCHANNELxx_vInputSDA();

    /* receive the bits -- starting with the MSB */
    ucCounter = 8u;

    do {
        HI2CxxCHANNELxx_vSetSCL();
        HI2CxxCHANNELxx_vBitDelayH();

        HI2CxxCHANNELxx_ucLastRx <<= 1u;

        if(HI2CxxCHANNELxx_bGetSDA() != 0u) {
            HI2CxxCHANNELxx_ucLastRx |= 1u;
        }
        HI2CxxCHANNELxx_vClrSCL();
        HI2CxxCHANNELxx_vBitDelayL();
        --ucCounter;
   } while(ucCounter > 0u);

    /* send ACK according to the stop flag */
    HI2CxxCHANNELxx_vOutputSDA();

    if(bStop != 0u) {
        /* no acknowledge */
        HI2CxxCHANNELxx_vSetSDA();
    } else {
        /* acknowledge */
        HI2CxxCHANNELxx_vClrSDA();
    }
    HI2CxxCHANNELxx_vWaitForSlave();
    HI2CxxCHANNELxx_vClrSCL();
    HI2CxxCHANNELxx_vSetSDA();
    HI2CxxCHANNELxx_vBitDelayL();

    if(bStop != 0u) {
        HI2CxxCHANNELxx_vMakeStopCondition();
    }
    /* call event handler */
    if(HI2CxxCHANNELxx_bEventEnabled != 0u) {
        HI2CxxCHANNELxx_vHandleEvent();
    }
    return HI2CxxCHANNELxx_ucLastRx;
}

void HI2CxxCHANNELxx_vSendStop(void) {
    /* Send stop-condition.  Changing of SDA during SCL = OUTPUT_HIGH is */
    /* only allowed while sending start- or stop-condition.  For generating */
    /* stop-condition both SDA and SCL must be LOW */
    HI2CxxCHANNELxx_vClrSCL();
    HI2CxxCHANNELxx_vClrSDA();
    HI2CxxCHANNELxx_vOutputSCL();
    HI2CxxCHANNELxx_vOutputSDA();

    HI2CxxCHANNELxx_vBitDelayL();           /* clock low period */
    HI2CxxCHANNELxx_vSetSCL();              /* change SCL edge to OUTPUT_HIGH level */
    HI2CxxCHANNELxx_vBitDelayH();           /* Stop condition setup time */
    HI2CxxCHANNELxx_vSetSDA();;             /* change SDA edge to OUTPUT_HIGH level */

    HI2CxxCHANNELxx_vInputSCL();
    HI2CxxCHANNELxx_vInputSDA();
}

bool HI2CxxCHANNELxx_bForceBusRelease(void) {
    uint16_t uiCounter;
    bool bSdaStatus;

    HI2CxxCHANNELxx_vOutputSCL();
    HI2CxxCHANNELxx_vInputSDA();

    for(uiCounter = 0u; uiCounter < 15u; uiCounter++) {
        HI2CxxCHANNELxx_vClrSCL();          /* security: send some clocks to free the bus-lines */
        HI2CxxCHANNELxx_vBitDelayL();          /* clock low period */
        HI2CxxCHANNELxx_vSetSCL();
        HI2CxxCHANNELxx_vBitDelayH();
    }
    HI2CxxCHANNELxx_vSendStop();

    /* check BUS-lines */
    for(uiCounter = 0u; uiCounter < 5u; uiCounter++) {
        bSdaStatus = HI2CxxCHANNELxx_bGetSDA();
        if(bSdaStatus != 0u) {
            break;
        }
    }
    return (bSdaStatus);
}

uint8_t HI2CxxCHANNELxx_readByte(uint8_t addr, bool stop) {
    uint8_t ret;
    ret = 0u;
    if(true == HI2CxxCHANNELxx_writeAddr(addr, true)) {
        if(true == HI2CxxCHANNELxx_bSetAddr(ucChipAddr | 0x01u)) { // read
            ret = HI2CxxCHANNELxx_vTriggerReceive(stop);
        }
    }
    return ret;
}
    
bool HI2CxxCHANNELxx_writeByte(uint8_t addr, bool stop, uint8_t data) {
    bool ret;
    ret = false;
    if(true == HI2CxxCHANNELxx_writeAddr(addr, false)) { // write
        if(true == HI2CxxCHANNELxx_bSetTxData(data, stop)) { // write address
            ret = true;
        }
    }
    return ret;
}

bool HI2CxxCHANNELxx_writeAddr(uint8_t addr, bool stop) {
    bool ret;
    ret = false;
    if(true == HI2CxxCHANNELxx_bSetAddr(ucChipAddr)) { // write
        bIsChippresent = true;
        if(true == HI2CxxCHANNELxx_bSetTxData(addr, stop)) { // write address
            ret = true;
        } else {
            bIsChippresent = false;
        }
    } else {
        bIsChippresent = false;
    }
    return ret;
}

void HI2CxxCHANNELxx_setChipAddress(uint8_t chipAddress) {
    ucChipAddr = (0xFEu & chipAddress);
}

uint8_t HI2CxxCHANNELxx_getChipAddress(void) {
    return ucChipAddr;
}

bool HI2CxxCHANNELxx_isChipPresent(void) {
    return bIsChippresent;
}
