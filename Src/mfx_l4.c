#include "mfx_l4.h"
#include "stm32l4xx.h"

#define MFX_WAKEUP_PIN0()     (GPIOA->BSRR |= GPIO_BSRR_BR4)
#define MFX_WAKEUP_PIN1()     (GPIOA->BSRR |= GPIO_BSRR_BS4)
//static e_mfx_status mfx_status = MFX_STATUS_NOT_INIT;
static e_mfx_status mfx_status = MFX_STATUS_SLEEP;
void mfx_initPort(void);
bool mfx_initConstValue(void);
void mfx_initIntEn(void);
void mfx_wakeup(void);
void mfx_delay(uint16_t time);
bool mfx_resetMFX(void);
bool mfx_setIrqOutPinPolarityAndOutputPin(uint8_t polarity, uint8_t type);
bool mfx_initConstValueSend(uint8_t addrToSend, uint8_t *buff, uint8_t size);

static int32_t mfx_result = 0u;
static bool mfx_isNewData = false;

 /**
  * @brief  Register address: chip IDs (R)
  */
#define MFXSTM32L152_REG_ADR_ID                 ((uint8_t)0x00)
 /**
  * @brief  Register address: chip FW_VERSION  (R)
  */
#define MFXSTM32L152_REG_ADR_FW_VERSION_MSB     ((uint8_t)0x01)
#define MFXSTM32L152_REG_ADR_FW_VERSION_LSB     ((uint8_t)0x00)
 /**
  * @brief  Register address: System Control Register (R/W)
  */
#define MFXSTM32L152_REG_ADR_SYS_CTRL           ((uint8_t)0x40)
      /**
      * @brief  MFXSTM32L152_REG_ADR_SYS_CTRL choices
      */
    #define MFXSTM32L152_SWRST                    ((uint8_t)0x80)
    #define MFXSTM32L152_STANDBY                  ((uint8_t)0x40)
    #define MFXSTM32L152_ALTERNATE_GPIO_EN        ((uint8_t)0x08) /* by the way if IDD and TS are enabled they take automatically the AF pins*/
    #define MFXSTM32L152_IDD_EN                   ((uint8_t)0x04)
    #define MFXSTM32L152_TS_EN                    ((uint8_t)0x02)
    #define MFXSTM32L152_GPIO_EN                  ((uint8_t)0x01)

 /**
  * @brief  Register address: Vdd monitoring (R)
  */
#define MFXSTM32L152_REG_ADR_VDD_REF_MSB        ((uint8_t)0x06)
#define MFXSTM32L152_REG_ADR_VDD_REF_LSB        ((uint8_t)0x07)
 /**
  * @brief  Register address: Error source
  */
#define MFXSTM32L152_REG_ADR_ERROR_SRC          ((uint8_t)0x03)
 /**
  * @brief  Register address: Error Message
  */
#define MFXSTM32L152_REG_ADR_ERROR_MSG          ((uint8_t)0x04)

 /**
  * @brief  Reg Addr IRQs: to config the pin that informs Main MCU that MFX events appear
  */
#define MFXSTM32L152_REG_ADR_MFX_IRQ_OUT        ((uint8_t)0x41)
    /**
      * @brief  MFXSTM32L152_REG_ADR_MFX_IRQ_OUT choices
      */
    #define MFXSTM32L152_OUT_PIN_TYPE_OPENDRAIN   ((uint8_t)0x00)
    #define MFXSTM32L152_OUT_PIN_TYPE_PUSHPULL    ((uint8_t)0x01)
    #define MFXSTM32L152_OUT_PIN_POLARITY_LOW     ((uint8_t)0x00)
    #define MFXSTM32L152_OUT_PIN_POLARITY_HIGH    ((uint8_t)0x02)
 /**
  * @brief  Reg Addr IRQs: to select the events which activate the MFXSTM32L152_IRQ_OUT signal
  */
#define MFXSTM32L152_REG_ADR_IRQ_SRC_EN         ((uint8_t)0x42)
 /**
  * @brief  Reg Addr IRQs: the Main MCU must read the IRQ_PENDING register to know the interrupt reason
  */
#define MFXSTM32L152_REG_ADR_IRQ_PENDING        ((uint8_t)0x08)
 /**
  * @brief  Reg Addr IRQs: the Main MCU must acknowledge it thanks to a writing access to the IRQ_ACK register
  */
#define MFXSTM32L152_REG_ADR_IRQ_ACK            ((uint8_t)0x44)
/**
  * @brief  Register address: Idd control register (R/W)
  */
#define MFXSTM32L152_REG_ADR_IDD_CTRL           ((uint8_t)0x80)
/**
  * @brief  Register address: Idd pre delay  register (R/W)
  */
#define MFXSTM32L152_REG_ADR_IDD_PRE_DELAY      ((uint8_t)0x81)
    #define MFXSTM32L152_IDD_PREDELAY_0_5_MS                ((uint8_t) 0x00)
    #define MFXSTM32L152_IDD_PREDELAY_20_MS                 ((uint8_t) 0x80)
    #define MFXSTM32L152_IDD_PREDELAY_UNIT                  ((uint8_t) 0x80)
    #define MFXSTM32L152_IDD_PREDELAY_VALUE                 ((uint8_t) 0x7F)



/**
  * @brief  Register address: Idd Shunt registers (R/W)
  */
#define MFXSTM32L152_REG_ADR_IDD_SHUNT0_MSB     ((uint8_t)0x82)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT0_LSB     ((uint8_t)0x83)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT1_MSB     ((uint8_t)0x84)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT1_LSB     ((uint8_t)0x85)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT2_MSB     ((uint8_t)0x86)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT2_LSB     ((uint8_t)0x87)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT3_MSB     ((uint8_t)0x88)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT3_LSB     ((uint8_t)0x89)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT4_MSB     ((uint8_t)0x8A)
#define MFXSTM32L152_REG_ADR_IDD_SHUNT4_LSB     ((uint8_t)0x8B)

/**
  * @brief  Register address: Idd ampli gain register (R/W)
  */
#define MFXSTM32L152_REG_ADR_IDD_GAIN_MSB       ((uint8_t)0x8C)
#define MFXSTM32L152_REG_ADR_IDD_GAIN_LSB       ((uint8_t)0x8D)

/**
  * @brief  Register address: Idd VDD min register (R/W)
  */
#define MFXSTM32L152_REG_ADR_IDD_VDD_MIN_MSB    ((uint8_t)0x8E)
#define MFXSTM32L152_REG_ADR_IDD_VDD_MIN_LSB    ((uint8_t)0x8F)


  /**
  * @brief  MFXSTM32L152_REG_ADR_ID choices
  */
#define MFXSTM32L152_ID_1                    ((uint8_t)0x7B)
#define MFXSTM32L152_ID_2                    ((uint8_t)0x79)


/** @defgroup IDD_Control_Register_Defines  IDD Control Register Defines
  * @{
  */
/**
  * @brief  IDD control register masks
  */
#define MFXSTM32L152_IDD_CTRL_REQ                               ((uint8_t)0x01)
#define MFXSTM32L152_IDD_CTRL_SHUNT_NB                          ((uint8_t)0x0E)
    #define MFXSTM32L152_IDD_SHUNT_NB_1                     ((uint8_t) 0x01)
    #define MFXSTM32L152_IDD_SHUNT_NB_2                     ((uint8_t) 0x02)
    #define MFXSTM32L152_IDD_SHUNT_NB_3                     ((uint8_t) 0x03)
    #define MFXSTM32L152_IDD_SHUNT_NB_4                     ((uint8_t) 0x04)
    #define MFXSTM32L152_IDD_SHUNT_NB_5                     ((uint8_t) 0x05)
#define MFXSTM32L152_IDD_CTRL_VREF_DIS                          ((uint8_t)0x40)
    #define MFXSTM32L152_IDD_VREF_AUTO_MEASUREMENT_ENABLE   ((uint8_t) 0x00)
    #define MFXSTM32L152_IDD_VREF_AUTO_MEASUREMENT_DISABLE  ((uint8_t) 0x70)
#define MFXSTM32L152_IDD_CTRL_CAL_DIS                           ((uint8_t)0x80)
    #define MFXSTM32L152_IDD_AUTO_CALIBRATION_ENABLE        ((uint8_t) 0x00)
    #define MFXSTM32L152_IDD_AUTO_CALIBRATION_DISABLE       ((uint8_t) 0x80)


void mfx_wakeup(void) {
    MFX_WAKEUP_PIN1();
    mfx_delay(1u);
    MFX_WAKEUP_PIN0();
}

void mfx_delay(uint16_t time) {
    TIM_delaySetTimer(DELAY_MFX_INIT, time);
    while(false == TIM_delayIsTimerDown(DELAY_MFX_INIT)) {            
        TIM_handleTask();
    };
}

#define MFXSTM32L152_ID_1                    ((uint8_t)0x7B)
#define MFXSTM32L152_ID_2                    ((uint8_t)0x79)
bool mfx_initForced(void) {
    bool ret;
    uint8_t value;
    uint8_t fwVersion[2];
    uint16_t temp;
    ret = false;
    // NOTE: I2C pins are configured in I2C_init function
    /* Initialize MFX */
    // reset MFX ( SYS_CTRL = SWRST )
    mfx_initPort();
    mfx_delay(10);
    if(true == mfx_resetMFX()) {
        // na adrese 0x00u musi vratit 0x7B alebo 0x79
        temp = HI2Cmfx_readByte(MFXSTM32L152_REG_ADR_ID, true);
        if((MFXSTM32L152_ID_1 == temp) || (MFXSTM32L152_ID_2 == temp)) {
            // vycita adresu 0x41 a modifikuje co prislo a posle spat ; nad adr. 0x41 vloz 0x03
            if(true == mfx_setIrqOutPinPolarityAndOutputPin(MFXSTM32L152_OUT_PIN_POLARITY_HIGH, MFXSTM32L152_OUT_PIN_TYPE_PUSHPULL)) {
                // read address 0x40u, mode 0x04u-MFXSTM32L152_IDD_EN; na adresu 0x40 vloz 0x04 
                if(MFXSTM32L152_IDD_EN != (HI2Cmfx_readByte(MFXSTM32L152_REG_ADR_SYS_CTRL, true) & MFXSTM32L152_IDD_EN)) {
                    // Set the Functionalities to be enabled
                    HI2Cmfx_writeByte(MFXSTM32L152_REG_ADR_SYS_CTRL, true, MFXSTM32L152_IDD_EN);
                }
                // Control register setting: number of shunts
                value = ((MFXSTM32L152_IDD_SHUNT_NB_4 << 1u) &  MFXSTM32L152_IDD_CTRL_SHUNT_NB); // 0x08 & 0x0E = 0x08
                value |= (MFXSTM32L152_IDD_VREF_AUTO_MEASUREMENT_ENABLE & MFXSTM32L152_IDD_CTRL_VREF_DIS); // 0
                value |= (MFXSTM32L152_IDD_AUTO_CALIBRATION_ENABLE & MFXSTM32L152_IDD_CTRL_CAL_DIS);       // 0
                // 0x08 write to address 0x80
                if(true == HI2Cmfx_writeByte(MFXSTM32L152_REG_ADR_IDD_CTRL, true, value)) {
                    value = (MFXSTM32L152_IDD_PREDELAY_20_MS & MFXSTM32L152_IDD_PREDELAY_UNIT); // 0x80 & 0x80 = 0x80
                    value |= (0x7Fu & MFXSTM32L152_IDD_PREDELAY_VALUE); // 0x80 | (0x7F & 0x7F) = 0xFF to address 0x81
                    // 0x81 zapis 0xFF
                    if(true == HI2Cmfx_writeByte(MFXSTM32L152_REG_ADR_IDD_PRE_DELAY, true, value)) {
                        if(true == mfx_initConstValue()) {
                            // read FW version (v2.6), not needed; read from address 0x01 two bytes
                            fwVersion[0] = HI2Cmfx_readByte(MFXSTM32L152_REG_ADR_FW_VERSION_MSB, false); // 2
                            fwVersion[1] = HI2Cmfx_vTriggerReceive(true);                                // 6
                            (void)fwVersion;
                            if(true == HI2Cmfx_writeByte(0x42u, true, 0x06u)) {
                                ret = true;
                            }
                        }
                    }
                }
            }
        }
    }
                        
                        
/*         
            
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
    }*/
    if(false == ret) {
        mfx_status = MFX_STATUS_NOT_INIT;
    }
    return ret;
}

/**
    Example of requesting an IDD measurement from the MFX
*/
void mfx_iddReqMeas(uint8_t predelay) {
    if(true == TIM_delayIsTimerDown(DELAY_MFX_INIT)) {
        TIM_delaySetTimer(DELAY_MFX_INIT, 5000u);
        mfx_status = MFX_STATUS_MEASUREMENT;
        mfx_isNewData = false;
    //predelay |= 0x80; // IDD_PRE_DELAY |= IDD_PREDELAY_20_MS
        //HI2Cmfx_writeByte(0x81u, true, (uint8_t)(predelay | 0x80u));
    // IDD_CTRL = ( ( 4 << 1 ) & IDD_CTRL_SHUNT_NB ) | IDD_CTRL_REQ
        mfx_initIntEn();
    HI2Cmfx_writeByte(0x80u, true, 0x09u); // request Idd measurement
}
}

int32_t mfx_getData(void) {
    return mfx_result;
}

int32_t mfx_iddGetMeas(void) {
    uint8_t temp[3];
    int32_t ret;
     
    ret = -1;
    // check for errors
    temp[0u] = HI2Cmfx_readByte(0x08u, true);
    if(temp[0u] & 0x04u) { // if ( REG_IRQ_PENGDING & IRQ_ERROR )
        //Idd_Init();
        if(mfx_status <= MFX_STATUS_SLEEP) {
            //mfx_initForced();
            //mfx_status = MFX_STATUS_ERROR;
        }
        //CURR_MEAS_POS();  // move cursor to current measurement position
        //USART_puts( USART1, "MFX ERROR" );
    } else { }{
        if(true == HI2Cmfx_writeByte(0x44u, 0x02u, true)) { // acknowledge Idd from MFX
        temp[0u] = HI2Cmfx_readByte(0x14u, false);
        temp[1u] = HI2Cmfx_vTriggerReceive(false);
        temp[2u] = HI2Cmfx_vTriggerReceive(true);
            ret = (temp[0]<<16) | (temp[1]<<8) | temp[2];
        }
        }
    return ret;
}

void mfx_convertToChar(uint8_t *buff, int32_t value) {
    uint8_t ret;
    ret = 'E';
    if(-1 != value) {
        if(value < 100) {
            value *= 100;
            ret = 'n';
        } else if(value < 100000) {
            value /= 10;
            ret = 'u';
        } else {
            value /= 10000;
            ret = 'm';
        }
    sprintf((char*)buff, "%ld%cA", (long)value, ret);
    }
}

bool mfx_initConstValueSend(uint8_t addrToSend, uint8_t *buff, uint8_t size) {
    bool ret;
    uint8_t i;
    ret = true;
     if(true == HI2Cmfx_writeByte(addrToSend, false, buff[0])) {
        for(i = 1u; i < size; i++) {
            if(false == HI2Cmfx_bSetTxData(buff[i], false)) {
                ret = false;
                break;
            }
        }
        if(false != ret) {
            ret = HI2Cmfx_bSetTxData(buff[i], true);
        }
    } else {
                ret = false;
    }
    return ret;
        }

bool mfx_initConstValue(void) {
    bool ret;
    uint8_t params[16];
    ret = false;
    params[0] = 0x08u; params[1] = 0xFFu;
    params[2] = 0x03u; params[3] = 0xE8u;     // SH0 = 1000 mohm
    params[4] = 0x00u; params[5] = 0x18u;     // SH1 = 24 ohm
    params[6] = 0x02u; params[7] = 0x6Cu;     // SH2 = 620 ohm
    params[8] = 0x00u; params[9] = 0x00u;     // SH3 = not included
    params[10] = 0x27u; params[11] = 0x10u;   // SH4 = 10,000 ohm
    params[12] = 0x13u; params[13] = 0x7Eu;   // Gain = 49.9 (4990)
    params[14] = 0x07u; params[15] = 0xD0u;   // 7D0 = 2000
    /*params[12] = 0x0B; params[13] = 0xB8;   // VDD_MIN = 3000 mV */

    if(true == mfx_initConstValueSend(0x80u, params, 15u)) {
        params[0] = 0x95u; params[1] = 0x95u;
        params[2] = 0x95u; params[3] = 0x00u;
        params[4] = 0xFFu;
        if(true == mfx_initConstValueSend(0x90u, params, 4u)) {
            params[0] = 0x64u;
            params[1] = 0x0Au;
            params[2] = 0x04u;
            if(true == mfx_initConstValueSend(0x96u, params, 2u)) {
                ret = true;
            }
        }
    }
    return ret;
}

#ifdef  MFX_DEFINE_COPY_PINS
void EXTI15_10_IRQHandler(void) {
    uint32_t temp;
    temp = EXTI->PR1;
    EXTI->PR1 = EXTI->IMR1; // zmaz vsetky masky ktore su povolene
    if(0u != (temp & EXTI_PR1_PIF10_Msk)) {
        //EXTI->PR1 |= EXTI_PR1_PIF10_Msk;
        if(0u != (GPIOB->IDR & GPIO_IDR_ID10)) {
            GPIOB->BSRR |= GPIO_BSRR_BS2;       // PB10 nastav na hodnotu PB2    clk
        } else {
            GPIOB->BSRR |= GPIO_BSRR_BR2;
        }
    }
    if(0u != (temp & EXTI_PR1_PIF11_Msk)) {
        //EXTI->PR1 |= EXTI_PR1_PIF11_Msk;
        if(0u != (GPIOB->IDR & GPIO_IDR_ID11)) {
            GPIOE->BSRR |= GPIO_BSRR_BS8;       // PB11 nastav na hodnotu PE8    data
        } else {
            GPIOE->BSRR |= GPIO_BSRR_BR8;
        }
    }
    if(0u != (temp & EXTI_PR1_PIF13_Msk)) {
        mfx_isNewData = true;
        EXTI->IMR1 &= ~EXTI_IMR1_IM13_Msk;
        if(0u != (GPIOC->IDR & GPIO_IDR_ID13)) {
            GPIOE->BSRR |= GPIO_BSRR_BS12;       // PE12 nastav na hodnotu PC13    MFX_IRQ
        } else {
            GPIOE->BSRR |= GPIO_BSRR_BR12;
        }
    }
}

void mfx_initPort(void) {
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOEEN_Msk);

    GPIOB->MODER &= ~(GPIO_MODER_MODE2_Msk);
    GPIOB->MODER |= (1u << GPIO_MODER_MODE2_Pos); // PB2 clk
    GPIOE->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE12_Msk);
    GPIOE->MODER |= ((1u << GPIO_MODER_MODE8_Pos) | (1u << GPIO_MODER_MODE12_Pos)); // PE8 data and PE12 MFX_IRQ

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[2] &= ~(SYSCFG_EXTICR3_EXTI10_Msk | SYSCFG_EXTICR3_EXTI11_Msk);
    SYSCFG->EXTICR[2] |= ((1u << SYSCFG_EXTICR3_EXTI10_Pos) | (1u << SYSCFG_EXTICR3_EXTI11_Pos)); // 1-PORT B 0
    RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
    EXTI->IMR1 |= (EXTI_IMR1_IM10_Msk | EXTI_IMR1_IM11_Msk);
    EXTI->RTSR1 |= (EXTI_RTSR1_RT10_Msk | EXTI_RTSR1_RT11_Msk);  // rising edge
    EXTI->FTSR1 |= (EXTI_FTSR1_FT10_Msk | EXTI_FTSR1_FT11_Msk);  // faling edge
    NVIC_EnableIRQ(EXTI15_10_IRQn);
#else
void mfx_initPort(void) {    
#endif

    mfx_isNewData = false;
    mfx_result = 0u;
    HI2Cmfx_vInit(0x84u);

    // Initialize GPIOs
    // PC13: recieve MFX_IRQ_OUT signal
    RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOCEN); // enable clocks
    GPIOA->MODER &= ~(GPIO_MODER_MODE4_Msk);
    GPIOA->MODER |= (1u << GPIO_MODER_MODE4_Pos); // MFX_WAKEUP
    GPIOC->MODER &= ~(GPIO_MODER_MODE13); // Input mode
    MFX_WAKEUP_PIN0();

    // Configure external interrupt on MFX_IRQ_OUT (PC13)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk; // ENABLE SYSTEM CONFIGURATION CONTROLLER CLOCK
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC; // PC13 is source for EXTI
    RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN_Msk;
    
    //EXTI->RTSR1 |= EXTI_RTSR1_RT13_Msk; // rising trigger enabled for input line 13
    EXTI->FTSR1 |= EXTI_FTSR1_FT13_Msk; // faling trigger enabled for input line 13

    // NOTE: I2C pins are configured in I2C_init function
    /* Initialize MFX */
    // reset MFX ( SYS_CTRL = SWRST )
    
    mfx_wakeup();
}

bool mfx_setIrqOutPinPolarityAndOutputPin(uint8_t polarity, uint8_t type) {
    bool ret;
    uint8_t temp;
    // 0x41
    temp = HI2Cmfx_readByte(MFXSTM32L152_REG_ADR_MFX_IRQ_OUT, true);
    temp &= ~(0x02 | 0x01);
    temp |= polarity;
    temp |= type;
    ret = HI2Cmfx_writeByte(MFXSTM32L152_REG_ADR_MFX_IRQ_OUT, true, temp);
    mfx_delay(1u);
    return ret;
}

bool mfx_resetMFX(void) {
    bool ret;
    // na adresu 0x40u poslem 0x80u
    ret = HI2Cmfx_writeByte(MFXSTM32L152_REG_ADR_SYS_CTRL, true, MFXSTM32L152_SWRST);
    mfx_delay(10u);
    return ret;
}

void mfx_initIntEn(void) {
    /* enable interrupts for external interrupt lines 4 - 15 */
    EXTI->PR1 |= EXTI_PR1_PIF13_Msk;   // clear pending interrupt (if it is pending)
    EXTI->IMR1 |= EXTI_IMR1_IM13_Msk; // interrupt request from line 13 masked
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    //NVIC_SetPriority(EXTI15_10_IRQn, IDD_INT_PRIO);
}

e_mfx_status mfx_handleTask(void) {
    switch(mfx_status) {
        case MFX_STATUS_SLEEP: break;
        case MFX_STATUS_NOT_INIT:
            // reset MFX ( SYS_CTRL = SWRST )
            mfx_initPort();
            if(true == mfx_resetMFX()) {
                TIM_delaySetTimer(DELAY_MFX_INIT, 100u);
                mfx_status = MFX_STATUS_INIT_REG_0x41;
            } else {
                mfx_status = MFX_STATUS_ERROR;
            }
            break;
        case MFX_STATUS_INIT_REG_0x41:
            if(true == TIM_delayIsTimerDown(DELAY_MFX_INIT)) {
                // IRQ pin -> push-pull, active high
                // ( IRQ_OUT = OUT_PIN_TYPE_PUSHPULL | OUT_PIN_POLARITY_HIGH )
                HI2Cmfx_writeByte(0x41u, true, 0x03u);
                TIM_delaySetTimer(DELAY_MFX_INIT, 2u);
                mfx_status = MFX_STATUS_INIT_REG_0x42;
            }
            break;
        case MFX_STATUS_INIT_REG_0x42:
            if(true == TIM_delayIsTimerDown(DELAY_MFX_INIT)) {
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
        case MFX_STATUS_DONE:
            mfx_status = MFX_STATUS_SLEEP;
            break;
        case MFX_STATUS_MEASUREMENT:
            if(true == mfx_isNewData) {
                mfx_isNewData = false;
                mfx_result = mfx_iddGetMeas();
                mfx_status = MFX_STATUS_DONE;
            } else if(true == TIM_delayIsTimerDown(DELAY_MFX_INIT)) {
                mfx_isNewData = true;
                EXTI->IMR1 &= ~EXTI_IMR1_IM13_Msk;
                //mfx_status = MFX_STATUS_ERROR;
            }
            break;
        case MFX_STATUS_INIT:
            mfx_status = MFX_STATUS_SLEEP;
            break;
        case MFX_STATUS_ERROR:
        default:
            mfx_status = MFX_STATUS_NOT_INIT;
            break;
    }
    return mfx_status;
}
