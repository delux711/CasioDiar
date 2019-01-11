#ifndef _BMP180_PRESSURE
#define _BMP180_PRESSURE

#include <stdint.h>
#include <stdbool.h>
#include "hi2c0.h"

typedef union _BMP180_calVal {
    uint16_t buff[11];
    struct _calBytes {
        int16_t AC1;
        int16_t AC2;
        int16_t AC3;
        uint16_t AC4;
        uint16_t AC5;
        uint16_t AC6;
        int16_t B1;
        int16_t B2;
        int16_t MB;
        int16_t MC;
        int16_t MD;
    } calBytes;
} BMP180_calVal;

typedef struct _BMP180 {
    BMP180_calVal calVal;
    int32_t UT;
    int32_t UP;
    int32_t X1;
    int32_t X2;
    int32_t X3;
    int32_t B3;
    uint32_t B4;
    int32_t B5;
    int32_t B6;
    uint32_t B7;
    uint8_t oss;
    int32_t T;
    int32_t P;
} BMP180;

typedef enum _BMP180_eOverSample {
    BMP180_eOverSampleTemperature = 0x2Eu,
    BMP180_eOverSampleMin4_5ms = 0x34u,
    BMP180_eOverSampleMedium7_5ms = 0x74u,
    BMP180_eOverSampleMedium13_5ms = 0xB4u,
    BMP180_eOverSampleMax25_5ms = 0xF4u
} BMP180_eOverSample;

typedef enum _BMP180_eState {
    BMP180_STATE_SLEEP,
    BMP180_STATE_NOT_INIT,
    BMP180_STATE_NOT_PRESENT,
    BMP180_STATE_MEASUREMENT_START,
    BMP180_STATE_TEMPERATURE_WHITE_TO_DONE,
    /* must be behind start*/
    BMP180_STATE_TEMPERATURE_READ1,
    BMP180_STATE_TEMPERATURE_READ2,
    /* must be behind end */
    BMP180_STATE_TEMPERATURE_CALCULATE,
    BMP180_STATE_PRESSURE_START,
    BMP180_STATE_PRESSURE_WHITE_TO_DONE,
    /* must be behind start*/
    BMP180_STATE_PRESSURE_READ1,
    BMP180_STATE_PRESSURE_READ2,
    BMP180_STATE_PRESSURE_READ3,
    /* must be behind end */
    BMP180_STATE_PRESSURE_CALCULATE
} BMP180_eState;

extern BMP180 sensorPresure;
extern bool BMP180_bBmp180present;

extern BMP180_eState BMP180_handleTask(void);
extern BMP180_eState BMP180_actualState(void);
extern void BMP180_startMeasurement(BMP180_eOverSample oss);
extern int32_t BMP180_getTemperature(void);
extern int32_t BMP180_getPressure(void);
extern uint8_t BMP180_getIdChip(void);
extern bool BMP180_isPresent(void);

extern void BMP180_readCalDataForced(void);
extern void BMP180_readTempForced(void);
extern void BMP180_readPressureAndTempForced(BMP180_eOverSample oss);
extern void BMP180_reset(void);
extern void BMP180_Init(void); // do not use if handleTask is used

#endif // _BMP180_PRESSURE
