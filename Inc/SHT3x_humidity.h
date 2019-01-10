#ifndef _SHT3X_H
#define _SHT3X_H
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx.h"
#include "hi2c0.h"

typedef enum _sht3x_status_t {
    SHT3X_STATUS_NOT_INIT,
} sht3x_status_t;

sht3x_status_t sht3x_handleTask(void);
extern uint8_t sht3x_getIdChip(void);

#endif // _SHT3X_H
