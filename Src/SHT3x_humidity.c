#include "SHT3x_humidity.h"
// SHT3x_

static SHT3x_status_t SHT3x_status = SHT3X_STATUS_NOT_INIT;
static bool SHT3x_bIsPresent = false;
static uint16_t SHT3x_uiLastTemperatureRaw = 0u;
static uint16_t SHT3x_uiLastHumidityRaw = 0u;

void SHT3x_delay(uint16_t time);
bool SHT3x_crcCheck(uint16_t crc);


void SHT3x_delay(uint16_t time) {
    TIM_delaySetTimer(DELAY_SHT3X_GLOBAL, time);
    while(false == TIM_delayIsTimerDown(DELAY_SHT3X_GLOBAL)) {            
        TIM_handleTask();
    };
}

void SHT3x_resetForced(void) {
    SHT3x_bIsPresent = HI2C0_writeByteForced(0x30u, true, 0xA2u); // send 0x30A2 to reset
}

void SHT3x_statusClearForced(void) {
    SHT3x_bIsPresent = HI2C0_writeByteForced(0x30u, true, 0x41u); // send 0X3041 to clear status register
}

void SHT3x_vInitForced(void) {
    SHT3x_uiLastTemperatureRaw = 0u;
    SHT3x_uiLastHumidityRaw = 0u;
    HI2C0_vInit(SHT3x_getIdChip());
    SHT3x_delay(2u); // 2ms wait after power reset
    SHT3x_resetForced();
    SHT3x_delay(2u); // 2ms wait after reset
    SHT3x_statusClearForced();
}

bool SHT3x_startMeasurementForced(void) {
    bool ret;
    uint8_t crc;
    ret = false;
    if(SHT3X_STATUS_SLEEP == SHT3x_status) {
        if(true == HI2C0_writeByteForced(0x2Cu, true, 0x10u)) { // 0x2C-enable clock stretching; 0x10 repeatability Low
            if(true == HI2C0_bSetAddr(SHT3x_getIdChip() | 0x01u)) { // read
                SHT3x_delay(20u);
                SHT3x_uiLastTemperatureRaw = (HI2C0_vTriggerReceive(false) << 8u);
                SHT3x_uiLastTemperatureRaw |= HI2C0_vTriggerReceive(false);
                crc = HI2C0_vTriggerReceive(false);
                if(true == SHT3x_crcCheck(crc)) {
                    SHT3x_uiLastHumidityRaw = (HI2C0_vTriggerReceive(false) << 8u);
                    SHT3x_uiLastHumidityRaw |= HI2C0_vTriggerReceive(false);
                    crc = HI2C0_vTriggerReceive(true);
                    if(true == SHT3x_crcCheck(crc)) {
                        ret = true;
                    }
                }
            }
        }
    }
    return ret;
}

void SHT3x_getMeasurementRaw(uint16_t *uiRawTemperature, uint16_t *uiRawHumidity) {
    *uiRawTemperature = SHT3x_uiLastTemperatureRaw;
    *uiRawHumidity = SHT3x_uiLastHumidityRaw;
}

float SHT3x_getTemperature(void) {
    return ((float)(175 * SHT3x_uiLastTemperatureRaw) / 65535u) - 45;
}

uint16_t SHT3x_getHumidity(void) {
    return (100u * SHT3x_uiLastHumidityRaw) / 65535u;
}

bool SHT3x_crcCheck(uint16_t crc) {
    return true;
}

bool SHT3x_isPresent(void) {
    return SHT3x_bIsPresent;
}

uint8_t SHT3x_getIdChip(void) {
    return 0x88u;
}

SHT3x_status_t SHT3x_actualState(void) {
    return SHT3x_status;
}

SHT3x_status_t SHT3x_handleTask(void) {
    switch(SHT3x_status) {
        case SHT3X_STATUS_NOT_INIT:
            SHT3x_vInitForced();
            SHT3x_status = SHT3X_STATUS_RESET_SEND;
            SHT3x_status = SHT3X_STATUS_SLEEP;
            break;
        case SHT3X_STATUS_RESET_SEND:
            
        default: break;
    }
    return SHT3x_status;
}
