#include "SHT3x_humidity.h"
// sht3x_

static sht3x_status_t sht3x_status = SHT3X_STATUS_NOT_INIT;
static bool sht3x_bIsPresent = false;

void sht3x_delay(uint16_t time);
bool sht3x_crcCheck(uint16_t crc);


void sht3x_delay(uint16_t time) {
    TIM_delaySetTimer(DELAY_SHT3X_GLOBAL, time);
    while(false == TIM_delayIsTimerDown(DELAY_SHT3X_GLOBAL)) {            
        TIM_handleTask();
    };
}

void sht3x_resetForced(void) {
    sht3x_bIsPresent = HI2C0_writeByte(0x30u, true, 0xA2u); // send 0x30A2 to reset
}

void sht3x_statusClearForced(void) {
    sht3x_bIsPresent = HI2C0_writeByte(0x30u, true, 0x41u); // send 0X3041 to clear status register
}

void sht3x_vInitForced(void) {
    HI2C0_vInit(sht3x_getIdChip());
    sht3x_delay(2u); // 2ms wait after power reset
    sht3x_resetForced();
    sht3x_delay(2u); // 2ms wait after reset
    sht3x_statusClearForced();
}

bool sht3x_getMeasurementForced(uint16_t *iTemperature, uint16_t *uiHumidity) {
    bool ret;
    uint8_t crc;
    ret = false;
    if(true == HI2C0_writeByte(0x2Cu, true, 0x10u)) { // 0x2C-enable clock stretching; 0x10 repeatability Low
        if(true == HI2C0_bSetAddr(sht3x_getIdChip() | 0x01u)) { // read
            //sht3x_delay(20u);
            *iTemperature = (HI2C0_vTriggerReceive(false) << 8u);
            *iTemperature |= HI2C0_vTriggerReceive(false);
            crc = HI2C0_vTriggerReceive(false);
            if(true == sht3x_crcCheck(crc)) {
                *uiHumidity = (HI2C0_vTriggerReceive(false) << 8u);
                *uiHumidity |= HI2C0_vTriggerReceive(false);
                crc = HI2C0_vTriggerReceive(true);
                if(true == sht3x_crcCheck(crc)) {
                    ret = true;
                }
            }
        }
    }
    return ret;
}

bool sht3x_crcCheck(uint16_t crc) {
    return true;
}

bool sht3x_isPresent(void) {
    return sht3x_bIsPresent;
}

uint8_t sht3x_getIdChip(void) {
    return 0x88u; // 0x44u;
}

sht3x_status_t sht3x_actualState(void) {
    return sht3x_status;
}

sht3x_status_t sht3x_handleTask(void) {
    switch(sht3x_status) {
        case SHT3X_STATUS_NOT_INIT:
            HI2C0_vInit(sht3x_getIdChip());
            sht3x_vInitForced();
            sht3x_status = SHT3X_STATUS_SLEEP;
            break;
        default: break;
    }
    return sht3x_status;
}
