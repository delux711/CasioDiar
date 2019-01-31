#ifndef _HI2C0_MANAGE_H
#define _HI2C0_MANAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "lcd.h"
#include "timerLib.h"
#include "hi2c0.h"
#include "BMP180_pressure.h"
#include "SHT3x_humidity.h"
#include "tm1638_modes.h"

void hi2c0m_handleTask(void);

#endif // _HI2C0_MANAGE_H
