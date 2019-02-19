#include "BMP180_pressure.h"

static BMP180 sensorPresure;
static bool BMP180_bBmp180present = false;
static BMP180_eState BMP180_state = BMP180_STATE_NOT_INIT;

void BMP180_calculateTemperature(void);
void BMP180_calculatePressure(void);
bool BMP180_startMeasurementForced(BMP180_eOverSample oss);

bool BMP180_isDoneSample(void) {
    return (0u == (0x20u & HI2C0_readByteForced(0xF4u, true)));
}

void BMP180_reset(void) {
    (void)HI2C0_writeByteForced(0xE0u, true, 0xB6u);   // 0xB6-reset sequence
}

void BMP180_readCalDataForced(void) {
    uint8_t i;
    if(true == HI2C0_writeAddrForced(0xAAu, true)) {
        if(true == HI2C0_bSetAddrForced(0xEFu)) { // read
            for(i = 0; i < 10u; i++) {
                sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
                sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(false);
            }
            sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
            sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(true);
        }
    }
}

bool BMP180_startMeasurementForced(BMP180_eOverSample oss) {
    bool ret;
    ret = HI2C0_writeByteForced(0xF4u, true, oss);
    if(true == ret) {
        while(false == BMP180_isDoneSample());
    }
    return ret;
}

void BMP180_readTempForced(void) {
    if(true == BMP180_startMeasurementForced(BMP180_eOverSampleTemperature)) {
        sensorPresure.UT = (int32_t)(HI2C0_readByteForced(0xF6u, false) << 8u); // address with temparature
        sensorPresure.UT |= HI2C0_vTriggerReceive(true);
        BMP180_calculateTemperature();
    }
}
    
void BMP180_readPressureAndTempForced(BMP180_eOverSample oss) {
    BMP180_readTempForced();
    if(true == BMP180_startMeasurementForced(oss)) {
        sensorPresure.oss = oss >> 6u;
        sensorPresure.UP =  (int32_t)(HI2C0_readByteForced(0xF6u, false) << 16u); // address with MSB pressure - address 0xF6
        sensorPresure.UP |= HI2C0_vTriggerReceive(false) << 8u;         // address with LSB pressure - address 0xF7
        sensorPresure.UP |= HI2C0_vTriggerReceive(true);                // address with XLSB pressure - address 0xF8
        BMP180_calculatePressure();
    }
}

void BMP180_calculateTemperature(void) {
    sensorPresure.X1 = sensorPresure.UT - sensorPresure.calVal.calBytes.AC6;
    sensorPresure.X1 *= sensorPresure.calVal.calBytes.AC5;
    sensorPresure.X1 /= 32768;  // 2^15
    sensorPresure.X2 = sensorPresure.calVal.calBytes.MC * 2048; // 2^11
    sensorPresure.X2 /= sensorPresure.X1 + sensorPresure.calVal.calBytes.MD;
    sensorPresure.B5 = sensorPresure.X1 + sensorPresure.X2;
    sensorPresure.T = (sensorPresure.B5 + 8u) / 16u;
}
    
void BMP180_calculatePressure(void) {
    sensorPresure.UP >>= (8u - sensorPresure.oss);
    sensorPresure.B6 = sensorPresure.B5 - 4000u;
    sensorPresure.X1 = sensorPresure.B6 * sensorPresure.B6;
    sensorPresure.X1 /= 4096u; // 2^12
    sensorPresure.X1 *= sensorPresure.calVal.calBytes.B2;
    sensorPresure.X1 /= 2048u; // 2^11
    sensorPresure.X2 = sensorPresure.calVal.calBytes.AC2 * sensorPresure.B6;
    sensorPresure.X2 /= 2048u; // 2^11
    sensorPresure.X3 = sensorPresure.X1 + sensorPresure.X2;
    sensorPresure.B3 = (((sensorPresure.calVal.calBytes.AC1 * 4 + sensorPresure.X3) << sensorPresure.oss) + 2u) / 4u;
    sensorPresure.X1 = (sensorPresure.calVal.calBytes.AC3 * sensorPresure.B6) / 8192u; // 2^13
    sensorPresure.X2 = (sensorPresure.calVal.calBytes.B1 * ((sensorPresure.B6 * sensorPresure.B6) / 4096u)) / 65536u; // 2^12; 2^16
    sensorPresure.X3 = ((sensorPresure.X1 + sensorPresure.X2) + 2) / 4;
    sensorPresure.B4 = sensorPresure.calVal.calBytes.AC4 * (uint32_t)(sensorPresure.X3 + 32768u) / 32768u; // 2^15
    sensorPresure.B7 = ((uint32_t)sensorPresure.UP - sensorPresure.B3) * (50000u >> sensorPresure.oss);
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

void BMP180_Init(void) {
    BMP180_bBmp180present = false;
    HI2C0_vInit(BMP180_getIdChip());
    if(0x55u == HI2C0_readByteForced(0xD0u, true)) {   // 0xD0u - Chip-id
        BMP180_bBmp180present = HI2C0_isChipPresent();
        BMP180_reset();
        BMP180_readCalDataForced();
    }
}

bool BMP180_isPresent(void) {
    return BMP180_bBmp180present;
}

int32_t BMP180_getTemperature(void) {
    return sensorPresure.T;
}

int32_t BMP180_getPressure(void) {
    return sensorPresure.P;
}

void BMP180_startMeasurement(BMP180_eOverSample oss) {
    if(BMP180_STATE_SLEEP == BMP180_state) {
        BMP180_state = BMP180_STATE_MEASUREMENT_START;
        sensorPresure.oss = (uint8_t) oss;
    }
}

BMP180_eState BMP180_handleTask(void) {
    switch(BMP180_state) {
        case BMP180_STATE_SLEEP:
            if(false == BMP180_isPresent()) {
                BMP180_state = BMP180_STATE_NOT_PRESENT;
            }
            break;
        case BMP180_STATE_NOT_PRESENT:
            if(true == TIM_delayIsTimerDown(DELAY_BMP180_REINIT)) {
                TIM_delaySetTimer(DELAY_BMP180_REINIT, BMP180_MAX_PRESENT_REINIT);
                BMP180_state = BMP180_STATE_NOT_INIT;
            }
            break;
        case BMP180_STATE_NOT_INIT:
            BMP180_Init();
            if(true == HI2C0_isChipPresent()) {
                BMP180_state = BMP180_STATE_SLEEP;
                BMP180_bBmp180present = HI2C0_isChipPresent();
            } else {
                BMP180_state = BMP180_STATE_NOT_PRESENT;
            }
            break;

        case BMP180_STATE_MEASUREMENT_START:
            if(true == HI2C0_writeByteForced(0xF4u, true, BMP180_eOverSampleTemperature)) {
                BMP180_state = BMP180_STATE_TEMPERATURE_WHITE_TO_DONE;
            } else {
                BMP180_bBmp180present = HI2C0_isChipPresent();
            }
            break;
        case BMP180_STATE_TEMPERATURE_WHITE_TO_DONE:
            if(true == BMP180_isDoneSample()) {
                BMP180_state = BMP180_STATE_TEMPERATURE_READ1;
            } else {
                BMP180_bBmp180present = HI2C0_isChipPresent();
            }
            break;
        case BMP180_STATE_TEMPERATURE_READ1:
            sensorPresure.UT = (int32_t)(HI2C0_readByteForced(0xF6u, false) << 8u); // address with temparature
            BMP180_state++;
            break;
        case BMP180_STATE_TEMPERATURE_READ2:
            sensorPresure.UT |= HI2C0_vTriggerReceive(true);
            BMP180_state = BMP180_STATE_TEMPERATURE_CALCULATE;
            break;
        case BMP180_STATE_TEMPERATURE_CALCULATE:
            BMP180_calculateTemperature();
            BMP180_state = BMP180_STATE_PRESSURE_START;
            break;

        case BMP180_STATE_PRESSURE_START:
            if(true == HI2C0_writeByteForced(0xF4u, true, sensorPresure.oss)) {
                BMP180_state = BMP180_STATE_PRESSURE_WHITE_TO_DONE;
            } else {
                BMP180_bBmp180present = HI2C0_isChipPresent();
            }
            break;
        case BMP180_STATE_PRESSURE_WHITE_TO_DONE:
            if(true == BMP180_isDoneSample()) {
                BMP180_state = BMP180_STATE_PRESSURE_READ1;
            } else {
                BMP180_bBmp180present = HI2C0_isChipPresent();
            }
            break;
        case BMP180_STATE_PRESSURE_READ1:
            sensorPresure.oss >>= 6u;
            sensorPresure.UP =  (int32_t)(HI2C0_readByteForced(0xF6u, false) << 16u); // address with MSB pressure - address 0xF6
            BMP180_state++;
            break;
        case BMP180_STATE_PRESSURE_READ2:
            sensorPresure.UP |= HI2C0_vTriggerReceive(false) << 8u;         // address with LSB pressure - address 0xF7
            BMP180_state++;
            break;
        case BMP180_STATE_PRESSURE_READ3:
            sensorPresure.UP |= HI2C0_vTriggerReceive(true);                // address with XLSB pressure - address 0xF8
            BMP180_state = BMP180_STATE_PRESSURE_CALCULATE;
            break;
        case BMP180_STATE_PRESSURE_CALCULATE:
            BMP180_calculatePressure();
            BMP180_state = BMP180_STATE_SLEEP;
            break;
        default:
            BMP180_state = BMP180_STATE_NOT_PRESENT;
            break;
    }
    if((false == BMP180_isPresent()) && (BMP180_STATE_NOT_INIT != BMP180_state)) {
        BMP180_state = BMP180_STATE_NOT_PRESENT;
    }
    return BMP180_state;
}

uint8_t BMP180_getIdChip(void) {
    return 0xEEu;
}

BMP180_eState BMP180_actualState(void) {
    return BMP180_state;
}
