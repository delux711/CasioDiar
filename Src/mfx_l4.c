#include "mfx_l4.h"
#include "stm32l4xx.h"

static e_mfx_status mfx_status = MFX_STATUS_NOT_INIT;
bool mfx_initPort(void);
bool mfx_initConstValue(void);
void mfx_initIntEn(void);

bool mfx_initForced(void) {
    bool ret;
    uint16_t temp;
    ret = false;
    // NOTE: I2C pins are configured in I2C_init function
    /* Initialize MFX */
    // reset MFX ( SYS_CTRL = SWRST )
    if(true == mfx_initPort()) {
        for(temp = 0u; temp < 10000; temp++);   //delay_ms(100);
        // IRQ pin -> push-pull, active high
        // ( IRQ_OUT = OUT_PIN_TYPE_PUSHPULL | OUT_PIN_POLARITY_HIGH )
        if(true == HI2Cmfx_writeByte(0x41u, true, 0x03u)) {
            for(temp = 0u; temp < 100; temp++);   //delay_ms(1);
            // IRQ source -> error and IDD ( IRQ_SRC_EN = IRQ_ERROR | IRQ_IDD )
            if(true == HI2Cmfx_writeByte(0x42u, true, 0x06)) {
                // Enable IDD function ( SYS_CTRL = IDD_EN )
                if(true == HI2Cmfx_writeByte(0x40u, true, 0x04)) {
                    // Assign shunt values, gain value, and min VDD value
                    if(true == mfx_initConstValue()) {
                        mfx_initIntEn();
                        ret = true;
                    }
                }
            }
        }
    }
    if(false == ret) {
        mfx_status = MFX_STATUS_NOT_INIT;
    }
    return ret;
}

/**
    Example of requesting an IDD measurement from the MFX
*/
void mfx_iddReqMeas(uint8_t predelay) {
    //predelay |= 0x80; // IDD_PRE_DELAY |= IDD_PREDELAY_20_MS
    HI2Cmfx_writeByte(0x81u, true, (uint8_t)(predelay | 0x80u));
    // IDD_CTRL = ( ( 4 << 1 ) & IDD_CTRL_SHUNT_NB ) | IDD_CTRL_REQ
    HI2Cmfx_writeByte(0x80u, true, 0x09u); // request Idd measurement
}


int32_t mfx_iddGetMeas(void) {
    int32_t ret;
    uint8_t temp[3];
     
    ret = -1;
    // check for errors
    temp[0u] = HI2Cmfx_readByte(0x08u, true);
    if(temp[0u] & 0x04u) { // if ( REG_IRQ_PENGDING & IRQ_ERROR )
        //Idd_Init();
        if(mfx_status <= MFX_STATUS_SLEEP) {
            mfx_initForced();
        }
        //CURR_MEAS_POS();  // move cursor to current measurement position
        //USART_puts( USART1, "MFX ERROR" );
    } else {
        temp[0u] = HI2Cmfx_readByte(0x14u, false);
        temp[1u] = HI2Cmfx_vTriggerReceive(false);
        temp[2u] = HI2Cmfx_vTriggerReceive(true);
        if(true == HI2Cmfx_writeByte(0x44u, 0x02u, true)) { // acknowledge Idd from MFX
            ret = (temp[0]<<16) + (temp[1]<<8) + temp[2];
        }
    }
    return ret;
}

bool mfx_initConstValue(void) {
    bool ret;
    uint8_t i;
    uint8_t params[14];
    ret = true;
    params[0] = 0x03; params[1] = 0xE8;     // SH0 = 1000 mohm
    params[2] = 0x00; params[3] = 0x18;     // SH1 = 24 ohm
    params[4] = 0x02; params[5] = 0x6C;     // SH2 = 620 ohm
    params[6] = 0x00; params[7] = 0x00;     // SH3 = not included
    params[8] = 0x27; params[9] = 0x10;     // SH4 = 10,000 ohm
    params[10] = 0x13; params[11] = 0x7E;   // Gain = 49.9 (4990)
    params[12] = 0x0B; params[13] = 0xB8;   // VDD_MIN = 3000 mV
    if(true == HI2Cmfx_writeByte(0x82u, false, params[0])) {
        for(i = 1u; i < 13u; i++) {
            if(false == HI2Cmfx_bSetTxData(params[i], false)) {
                ret = false;
                break;
            }
        }
        (void)HI2Cmfx_bSetTxData(params[i], true);
    } else {
        ret = false;
    }
    return ret;
}

bool mfx_initPort(void) {
    HI2Cmfx_vInit(0x84u);

    /* Initialize GPIOs */
    // PC13: recieve MFX_IRQ_OUT signal
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; // enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // ENABLE SYSTEM CONFIGURATION CONTROLLER CLOCK
    GPIOC->MODER &= ~(GPIO_MODER_MODE13); // Input mode
    // Configure external interrupt on MFX_IRQ_OUT (PC13)
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // PC13 is source for EXTI
    EXTI->IMR1 |= EXTI_IMR1_IM13; // interrupt request from line 13 masked
    EXTI->RTSR1 |= EXTI_RTSR1_RT13; // rising trigger enabled for input line 13

    // NOTE: I2C pins are configured in I2C_init function
    /* Initialize MFX */
    // reset MFX ( SYS_CTRL = SWRST )
    return HI2Cmfx_writeByte(0x40u, true, 0x80u);
}

void mfx_initIntEn(void) {
    /* enable interrupts for external interrupt lines 4 - 15 */
    EXTI->PR |= EXTI_PR_PR13; // clear pending interrupt (if it is pending)
    NVIC_EnableIRQ( EXTI4_15_IRQn );
    NVIC_SetPriority( EXTI4_15_IRQn, IDD_INT_PRIO );
}

e_mfx_status mfx_handleTask(void) {
    switch(mfx_status) {
        case MFX_STATUS_SLEEP: break;
        case MFX_STATUS_NOT_INIT:
            // reset MFX ( SYS_CTRL = SWRST )
            if(true == mfx_initPort()) {
                TIM_delaySetTimer(DELAY_MFX_INIT, 100u);
                mfx_status = MFX_STATUS_INIT_REG_0x41;
            } else {
                mfx_status = MFX_STATUS_ERROR;
            }
            break;
        case MFX_STATUS_INIT_REG_0x41:
            if(true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW)) {
                // IRQ pin -> push-pull, active high
                // ( IRQ_OUT = OUT_PIN_TYPE_PUSHPULL | OUT_PIN_POLARITY_HIGH )
                HI2Cmfx_writeByte(0x41u, true, 0x03u);
                TIM_delaySetTimer(DELAY_MFX_INIT, 2u);
                mfx_status = MFX_STATUS_INIT_REG_0x42;
            }
            break;
        case MFX_STATUS_INIT_REG_0x42:
            if(true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW)) {
                // IRQ source -> error and IDD ( IRQ_SRC_EN = IRQ_ERROR | IRQ_IDD )
                HI2Cmfx_writeByte(0x42u, true, 0x06);
                mfx_status = MFX_STATUS_INIT_REG_IDD_EN;
            }
            break;
        case MFX_STATUS_INIT_REG_IDD_EN:
            // Enable IDD function ( SYS_CTRL = IDD_EN )
            HI2Cmfx_writeByte(0x40u, true, 0x04);
            mfx_status = MFX_STATUS_INIT_REG_SET_CONST_VALUE;
            break;
        case MFX_STATUS_INIT_REG_SET_CONST_VALUE:
            // Assign shunt values, gain value, and min VDD value
            if(true == mfx_initConstValue()) {
                mfx_initIntEn();
                mfx_status = MFX_STATUS_INIT;
            } else {
                mfx_status = MFX_STATUS_ERROR;
            }
            break;
        case MFX_STATUS_INIT:
            mfx_status = MFX_STATUS_SLEEP;
            break;
    }
    return mfx_status;
}
