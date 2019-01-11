#include "hi2c0_manage.h"
#include "SHT3x_humidity.h"

#define HI2CM_STATE_INIT_BMP180     (0x01u)
#define HI2CM_STATE_INIT_SHT3X      (0x02u)

static uint8_t hi2c0m_stateInit = (HI2CM_STATE_INIT_BMP180 | HI2CM_STATE_INIT_SHT3X);
static bool hi2c0m_tempShow = false;
static bool hi2c0m_humidityShow = false;
static uint16_t iTemp = 0u;
static uint16_t uiHumid = 0u;

void hi2c0m_handleTask(void) {
    BMP180_eState bmpState;
    uint8_t buff[10];
    uint32_t temp;
    
    if(0u == hi2c0m_stateInit) {
        if(BMP180_STATE_SLEEP == BMP180_actualState()) {
            if(true == TIM_delayIsTimerDown(DELAY_SHT3X_SHOW)) {
                TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 4000u);
                TIM_delaySetTimer(DELAY_SHT3X_SHOW, 1500u);
                HI2C0_setChipAddress(sht3x_getIdChip());
                if(true == sht3x_getMeasurementForced(&iTemp, &uiHumid)) {
                    hi2c0m_humidityShow = true;
                }
            }
            if(true == hi2c0m_humidityShow) {
                hi2c0m_humidityShow = false;
                temp = (175*(iTemp/(0xFFFFu)));
                temp -= 45;
                sprintf((char*)buff, "T:%d°C", temp);
                LCD_GLASS_DisplayString((uint8_t*) buff);
            }
        }
        if(SHT3X_STATUS_SLEEP == sht3x_actualState()) {
            HI2C0_setChipAddress(BMP180_getIdChip());
            bmpState = BMP180_handleTask();
            if(true == BMP180_isPresent()) {
                if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW) == true) {
                    TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 4000u);
                    //BMP180_readPressureAndTempForced(BMP180_eOverSampleMax25_5ms);
                    BMP180_startMeasurement(BMP180_eOverSampleMax25_5ms);
                    hi2c0m_tempShow = true;
                }
                if((true == hi2c0m_tempShow) && (BMP180_STATE_SLEEP == bmpState)) {
                    hi2c0m_tempShow = false;
                    TIM_delaySetTimer(DELAY_MAIN_LCD_PRESSURE_SHOW, 1000u);
                    TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
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
        }
    } else {
        if(0u != (HI2CM_STATE_INIT_BMP180 & hi2c0m_stateInit)) {
            if(BMP180_STATE_SLEEP == BMP180_handleTask()) {
                hi2c0m_stateInit &= ~HI2CM_STATE_INIT_BMP180;
            }
        } else if(0u != (HI2CM_STATE_INIT_SHT3X & hi2c0m_stateInit)) {
            if(SHT3X_STATUS_SLEEP == sht3x_handleTask()) {
                hi2c0m_stateInit &= ~HI2CM_STATE_INIT_SHT3X;
            }
        }
    }
}
