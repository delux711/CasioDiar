#include "BMP180_pressure.h"
#include "../hi2c/hi2c.h"

static BMP180 sensorPresure;
static bool BMP180_bBmp180present = false;
static BMP180_eState BMP180_state = BMP180_STATE_NOT_INIT;

bool BMP180_isDoneSample(void) {
    return (0u == (0x20u & HI2C_readByte(0xF4u, true)));
}

bool HI2C_writeAddr(uint8_t addr, bool stop) {
    bool ret;
    ret = false;
    if(true == HI2C0_bSetAddr(0xEFu & 0xFE)) { // write
        if(true == HI2C0_bSetTxData(addr, stop)) { // write address
            ret = true;
        } else {
            BMP180_bBmp180present = false;
            BMP180_state = BMP180_STATE_NOT_PRESENT;
        }
    } else {
        BMP180_bBmp180present = false;
        BMP180_state = BMP180_STATE_NOT_PRESENT;
    }
    return ret;
}

bool HI2C_writeByte(uint8_t addr, bool stop, uint8_t data) {
    bool ret;
    ret = false;
    if(true == HI2C_writeAddr(addr, false)) { // write
        if(true == HI2C0_bSetTxData(data, stop)) { // write address
            ret = true;
        }
    }
    return ret;
}

void BMP180_reset(void) {
    (void)HI2C_writeByte(0xE0u, true, 0xB6u);   // 0xB6-reset sequence
}
        
        
uint8_t HI2C_readByte(uint8_t addr, bool stop) {
    uint8_t ret;
    ret = 0u;
    if(true == HI2C_writeAddr(addr, true)) {
        if(true == HI2C0_bSetAddr(0xEFu)) { // read
            ret = HI2C0_vTriggerReceive(stop);
        }
    }
    return ret;
}
/*
void BMP180_readCalData(void) {
    uint8_t i;
    if(true == HI2C0_bSetAddr(0xEFu & 0xFE)) { // write
        if(true == HI2C0_bSetTxData(0xAAu, true)) { // write address with calibration data
            if(true == HI2C0_bSetAddr(0xEFu)) { // read
                for(i = 0; i < 10u; i++) {
                    sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
                    sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(false);
                }
                sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
                sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(true);
            }
        }
    }
}
*/
void BMP180_readCalData(void) {
    uint8_t i;
    if(true == HI2C_writeAddr(0xAAu, true)) {
        if(true == HI2C0_bSetAddr(0xEFu)) { // read
            for(i = 0; i < 10u; i++) {
                sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
                sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(false);
            }
            sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
            sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(true);
        }
    }
}
bool BMP180_startMeasurement(BMP180_eOverSample oss) {
    bool ret;
    ret = HI2C_writeByte(0xF4u, true, oss);
    if(true == ret) {
        while(false == BMP180_isDoneSample());
    }
    return ret;
}
void BMP180_readTemp(void) {
    if(true == BMP180_startMeasurement(BMP180_eOverSampleTemperature)) {
        sensorPresure.UT = (long)(HI2C_readByte(0xF6u, false) << 8u); // address with temparature
        sensorPresure.UT |= HI2C0_vTriggerReceive(true);
        sensorPresure.X1 = sensorPresure.UT - sensorPresure.calVal.calBytes.AC6;
        sensorPresure.X1 *= sensorPresure.calVal.calBytes.AC5;
        sensorPresure.X1 /= 32768;  // 2^15
        sensorPresure.X2 = sensorPresure.calVal.calBytes.MC * 2048; // 2^11
        sensorPresure.X2 /= sensorPresure.X1 + sensorPresure.calVal.calBytes.MD;
        sensorPresure.B5 = sensorPresure.X1 + sensorPresure.X2;
        sensorPresure.T = (sensorPresure.B5 + 8u) / 16u;
    }
}
void BMP180_readPressureAndTemp(BMP180_eOverSample oss) {
    BMP180_readTemp();
    if(true == BMP180_startMeasurement(oss)) {
        sensorPresure.UP =  (long)(HI2C_readByte(0xF6u, false) << 16u); // address with MSB pressure - address 0xF6
        sensorPresure.UP |= HI2C0_vTriggerReceive(false) << 8u;         // address with LSB pressure - address 0xF7
        sensorPresure.UP |= HI2C0_vTriggerReceive(true);                // address with XLSB pressure - address 0xF8
        oss >>= 6u;
        sensorPresure.UP >>= 8u - oss;
        sensorPresure.B6 = sensorPresure.B5 - 4000u;
        sensorPresure.X1 = sensorPresure.B6 * sensorPresure.B6;
        sensorPresure.X1 /= 4096u; // 2^12
        sensorPresure.X1 *= sensorPresure.calVal.calBytes.B2;
        sensorPresure.X1 /= 2048u; // 2^11
        sensorPresure.X2 = sensorPresure.calVal.calBytes.AC2 * sensorPresure.B6;
        sensorPresure.X2 /= 2048u; // 2^11
        sensorPresure.X3 = sensorPresure.X1 + sensorPresure.X2;
        sensorPresure.B3 = (((sensorPresure.calVal.calBytes.AC1 * 4 + sensorPresure.X3) << oss) + 2u) / 4u;
        sensorPresure.X1 = (sensorPresure.calVal.calBytes.AC3 * sensorPresure.B6) / 8192u; // 2^13
        sensorPresure.X2 = (sensorPresure.calVal.calBytes.B1 * ((sensorPresure.B6 * sensorPresure.B6) / 4096u)) / 65536u; // 2^12; 2^16
        sensorPresure.X3 = ((sensorPresure.X1 + sensorPresure.X2) + 2u) / 2u;
        sensorPresure.B4 = sensorPresure.calVal.calBytes.AC4 * (uint32_t)(sensorPresure.X3 + 32768u) / 32768u; // 2^15
        sensorPresure.B7 = ((uint32_t)sensorPresure.UP - sensorPresure.B3) * (50000u >> oss);
        if(sensorPresure.B7 < 0x80000000u) {
            sensorPresure.P = (sensorPresure.B7 * 2u) / sensorPresure.B4;
        } else {
            sensorPresure.P = (sensorPresure.B7 / sensorPresure.B4) * 2u;
        }
        sensorPresure.X1 = (sensorPresure.P / 256u) * (sensorPresure.P / 256);  // 2^8
        sensorPresure.X1 = (sensorPresure.X1 * 3038u) / 65536u; // 2^16
        sensorPresure.X2 = (-7357 * sensorPresure.P) / 65536;   // 2^16
        sensorPresure.P = sensorPresure.P + (sensorPresure.X1 + sensorPresure.X2 + 3791) / 16;  // 2^4
    }
}

void BMP180_Init(void) {
    BMP180_bBmp180present = false;
    HI2C0_vInit(0u);
    if(0x55u == HI2C_readByte(0xD0u, true)) {   // 0xD0u - Chip-id
        BMP180_bBmp180present = true;
        BMP180_reset();
        BMP180_readCalData();
    }
}

bool BMP180_isPresent(void) {
    return BMP180_bBmp180present;
}

long BMP180_getTemperature(void) {
    return sensorPresure.T;
}

long BMP180_getPressure(void) {
    return sensorPresure.P;
}

BMP180_eState BMP180_handleTask(void) {
    switch(BMP180_state) {
        case BMP180_STATE_SLEEP:
            break;
        case BMP180_STATE_NOT_PRESENT:
            break;
        case BMP180_STATE_NOT_INIT:
            BMP180_Init();
            if(true == BMP180_bBmp180present) {
                BMP180_state = BMP180_STATE_SLEEP;
            }
            break;
        
    }
    return BMP180_state;
}
