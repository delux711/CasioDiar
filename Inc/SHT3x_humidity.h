#ifndef _SHT3X_H
#define _SHT3X_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx.h"
#include "hi2c0.h"
#include "timerLib.h"

#define SHT3X_MAX_PRESENT_REINIT   (100u)

typedef enum _SHT3x_status_t {
    SHT3X_STATUS_SLEEP,
    SHT3X_STATUS_NOT_INIT,
    SHT3X_STATUS_NOT_PRESENT,
    SHT3X_STATUS_RESET_SEND,
    SHT3X_STATUS_RESET_WAIT,
    /* must be behind start*/
    SHT3X_STATUS_SEND_COMMAND,
    SHT3X_STATUS_SEND_COMMAND_MSB,
    SHT3X_STATUS_SEND_COMMAND_LSB,
    /* must be behind end */
    
    SHT3X_STATUS_READ_STATUS_I2C_WRITE,
    SHT3X_STATUS_READ_STATUS_COMMAND_MSB,
    SHT3X_STATUS_READ_STATUS_COMMAND_LSB,
    SHT3X_STATUS_READ_STATUS_I2C_READ,
    SHT3X_STATUS_READ_STATUS_READ_MSB,
    SHT3X_STATUS_READ_STATUS_READ_LSB,
    SHT3X_STATUS_READ_STATUS_READ_CRC,
    SHT3X_STATUS_START_MEASUREMENT1,
    SHT3X_STATUS_START_MEASUREMENT2,
} SHT3x_status_t;

extern SHT3x_status_t SHT3x_handleTask(void);
extern SHT3x_status_t SHT3x_actualState(void);
extern bool SHT3x_isPresent(void);
extern uint8_t SHT3x_getIdChip(void);
extern void SHT3x_getMeasurementRaw(uint16_t *iTemperature, uint16_t *uiHumidity);
extern float SHT3x_getTemperature(void);
extern uint16_t SHT3x_getHumidity(void);

extern void SHT3x_vInitForced(void);
extern bool SHT3x_startMeasurementForced(void);
extern void SHT3x_resetForced(void);
extern void SHT3x_statusClearForced(void);

#endif // _SHT3X_H
