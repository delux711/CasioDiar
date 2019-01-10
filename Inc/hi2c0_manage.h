#ifndef _SHT3X_H
#define _SHT3X_H
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "lcd.h"
#include "timerLib.h"
#include "hi2c0.h"
#include "BMP180_pressure.h"

void hi2c0m_handleTask(void);

#endif // _SHT3X_H
