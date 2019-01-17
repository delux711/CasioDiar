#include "hi2c0_manage.h"
#include "SHT3x_humidity.h"
// hi2c0m_

#define HI2CM_STATE_INIT_BMP180     (0x01u)
#define HI2CM_STATE_INIT_SHT3X      (0x02u)

static uint8_t hi2c0m_stateInit = (HI2CM_STATE_INIT_BMP180 | HI2CM_STATE_INIT_SHT3X);
static uint8_t hi2c0m_showStep = 0u;
static bool hi2c0m_tempShow = false;
static bool hi2c0m_humidityShow = false;
static uint16_t iTemp = 0u;
static uint16_t uiHumid = 0u;

void hi2c0m_handleTask(void) {
    BMP180_eState bmpState;
    uint8_t buff[15];
    uint32_t hum;
    float tempBmp;
    float temp;
    
    if((0u == hi2c0m_stateInit) && (true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW))) {
        if((BMP180_STATE_SLEEP == BMP180_actualState()) && (false == hi2c0m_humidityShow)) {
            HI2C0_setChipAddress(sht3x_getIdChip());
            if(true == sht3x_getMeasurementForced(&iTemp, &uiHumid)) {
                hi2c0m_humidityShow = true;
                BMP180_startMeasurement(BMP180_eOverSampleMax25_5ms);
            }
        } else if((SHT3X_STATUS_SLEEP == sht3x_actualState()) && (false == hi2c0m_tempShow)) {
            HI2C0_setChipAddress(BMP180_getIdChip());
            bmpState = BMP180_handleTask();
            if(true == BMP180_isPresent()) {
                if(BMP180_STATE_SLEEP == bmpState) {
                    hi2c0m_tempShow = true;
                }
            }
        } else if((true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW)) &&
                    (true == hi2c0m_humidityShow) && (true == hi2c0m_tempShow)) {
            TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 800u);
            TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 800u);
            switch(hi2c0m_showStep++) {
                case 0: 
                    temp = ((float)(175*iTemp)/0xFFFFu) - 45;
                    sprintf((char*)buff, "T:%.2fC", temp);
                    break;
                case 1:
                    hum = (100u * uiHumid) / 65535u;
                    sprintf((char*)buff, "H:%d%%", hum);
                    break;
                case 2: 
                    tempBmp = (float)BMP180_getTemperature() / 10;
                    //sprintf((char*)buff, "T:%d.%dC", (int8_t)(BMP180_getTemperature() / 10u), tempBmp);
                    sprintf((char*)buff, "T:%.1fC", tempBmp);
                    break;
                case 3: sprintf((char*)buff, "P:%d", (int16_t)BMP180_getPressure()); // no break
                default:
                    hi2c0m_showStep = 0u;
                    hi2c0m_humidityShow = false;
                    hi2c0m_tempShow = false;
                    TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 3000u);
                    break;
            }
            LCD_GLASS_DisplayString((uint8_t*) buff);
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
