#include "hi2c0_manage.h"

#define HI2CM_STATE_INIT_BMP180     (0x01u)
#define HI2CM_STATE_INIT_SHT3X      (0x02u)

static uint8_t hi2c0m_stateInit = (HI2CM_STATE_INIT_BMP180); // && HI2CM_STATE_INIT_SHT3X);

void hi2c0m_handleTask(void) {
    BMP180_eState bmpState;
    uint8_t buff[10];
    
    if(0u == hi2c0m_stateInit) {
        HI2C0_setChipAddress(BMP180_getIdChip());
        if(true == BMP180_isPresent()) {
            if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW) == true) {
                TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 4000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_PRESSURE_SHOW, 1000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                BMP180_readPressureAndTempForced(BMP180_eOverSampleMax25_5ms);
                sprintf((char*)buff, "%d.%d°C", (int8_t)(BMP180_getTemperature() / 10u), (BMP180_getTemperature() / 100u));
                LCD_GLASS_DisplayString((uint8_t*) buff);
            }
            if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_PRESSURE_SHOW) == true) {
                TIM_delaySetTimer(DELAY_MAIN_LCD_PRESSURE_SHOW, 5000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                sprintf((char*)buff, "%d", (int16_t)BMP180_getPressure());
                LCD_GLASS_DisplayString((uint8_t*) buff);
            }
        }
    } else {
        if(true == (HI2CM_STATE_INIT_BMP180 & hi2c0m_stateInit)) {
            if(BMP180_STATE_SLEEP == BMP180_handleTask()) {
                hi2c0m_stateInit &= ~HI2CM_STATE_INIT_BMP180;
            }// else if(
        }
    }
}
