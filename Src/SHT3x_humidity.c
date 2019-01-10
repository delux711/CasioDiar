#include "SHT3x_humidity.h"

static sht3x_status_t sht3x_status = SHT3X_STATUS_NOT_INIT;

uint8_t sht3x_getIdChip(void) {
    return 0;
}

sht3x_status_t sht3x_handleTask(void) {
    uint8_t i, ch;
    bool pin;
    switch(sht3x_status) {
        case SHT3X_STATUS_NOT_INIT:
            HI2C0_vInit();
            break;
        
        default: break;
    }
    return sht3x_status;
}
