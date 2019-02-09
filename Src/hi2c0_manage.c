#include "hi2c0_manage.h"
// hi2c0m_

#define HI2CM_STATE_INIT_BMP180     (0x01u)
#define HI2CM_STATE_INIT_SHT3X      (0x02u)

static uint8_t hi2c0m_stateInit = (HI2CM_STATE_INIT_BMP180 | HI2CM_STATE_INIT_SHT3X);
static uint8_t hi2c0m_showStep = 0u;
static uint8_t buffTm1638[10];
static bool hi2c0m_tempShow = false;
static bool hi2c0m_humidityShow = false;

void hi2c0m_prepareTM1638Temperature(uint8_t *buff);

void hi2c0m_prepareTM1638Temperature(uint8_t *buff) {
    uint8_t i, ch;
    for(i = 2u; i < (sizeof(buffTm1638) + 2u); i++) {
        ch = buff[i];
        buffTm1638[i-2u] = ch;
        if('C' == ch) {
            break;
        }
    }
    if(8u == i) {
        if('0' == buffTm1638[5]) {
            buffTm1638[5] = 255u; // '°' -> 130.0°C; -15.2°C
        }                         // else   -15.25C; 134.99C
    } else if((7u == i) && ('0' == buffTm1638[4])) {
        buffTm1638[4] = 255u; // '°' -> 25.0°C; -5.2°C
        buffTm1638[6] = ' ';
    } else if(6u == i) {
        if('0' == buffTm1638[3]) {
            buffTm1638[3] = 255u; // '°' -> 5.2°C
            buffTm1638[5] = ' ';
        } else {
            buffTm1638[4] = 255u; // '°' -> 5.25°C
            buffTm1638[5] = 'C';
        }
        buffTm1638[6] = ' ';
    } else {
        buffTm1638[5] = 255u; // '°' -> 25.25°C; -5.25°C
        buffTm1638[6] = 'C';
    }
}

void hi2c0m_handleTask(void) {
    BMP180_eState bmpState;
    uint8_t buff[15];
    float tempBmp;

    if((0u == hi2c0m_stateInit) && (true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW))) {
        if((BMP180_STATE_SLEEP == BMP180_actualState()) && (false == hi2c0m_humidityShow)) {
            HI2C0_setChipAddress(SHT3x_getIdChip());
            if(true == SHT3x_startMeasurementForced()) {
                hi2c0m_humidityShow = true;
                BMP180_startMeasurement(BMP180_eOverSampleMax25_5ms);
            }
        } else if((SHT3X_STATUS_SLEEP == SHT3x_actualState()) && (false == hi2c0m_tempShow)) {
            HI2C0_setChipAddress(BMP180_getIdChip());
            bmpState = BMP180_handleTask();
            if(true == BMP180_isPresent()) {
                if(BMP180_STATE_SLEEP == bmpState) {
                    hi2c0m_tempShow = true;
                }
            }
        } else if((true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW)) &&
                    (true == hi2c0m_humidityShow) && (true == hi2c0m_tempShow)) {
            TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 500u);
            TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 500u);
            switch(hi2c0m_showStep++) {
                case 0: 
                    TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 1000u);
                    TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                    sprintf((char*)buff, "T:%.2fC", SHT3x_getTemperature());
                    if(TMM_STATUS_MODE_4_TEMP == TMM_getState()) {
                        hi2c0m_prepareTM1638Temperature(buff);
                    }
                    break;
                case 1:
                    TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 1000u);
                    TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                    sprintf((char*)buff, "H:%d%%", SHT3x_getHumidity());
                    if(TMM_STATUS_MODE_4_TEMP == TMM_getState()) {
                        buffTm1638[7] = buff[2];
                        buffTm1638[8] = buff[3];
                        buffTm1638[9] = '\0';
                        tm1638_show(buffTm1638);
                    }
                    break;
                case 2: 
                    tempBmp = (float)BMP180_getTemperature() / 10;
                    sprintf((char*)buff, "T:%.1fC", tempBmp);
                    break;
                case 3: sprintf((char*)buff, "P:%d", (int16_t)BMP180_getPressure()); // no break
                default:
                    hi2c0m_showStep = 0u;
                    hi2c0m_humidityShow = false;
                    hi2c0m_tempShow = false;
                    TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 4000u);
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
            if(SHT3X_STATUS_SLEEP == SHT3x_handleTask()) {
                hi2c0m_stateInit &= ~HI2CM_STATE_INIT_SHT3X;
            }
        }
    }
    TMM_handleTask();
}
