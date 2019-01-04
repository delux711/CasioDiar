#ifndef _TM1638_MODES_H
#define _TM1638_MODES_H
#include <stdint.h>
#include <stdbool.h>

#include "tm1638.h"
#include "timerLib.h"

typedef enum _TMM_tStatus {
    TMM_STATUS_NOT_INIT,
    TMM_STATUS_MODE_1,
    TMM_STATUS_MODE_2,
    TMM_STATUS_MODE_3,
    TMM_STATUS_MODE_3_NUM
} TMM_tStatus;

extern void TMM_handleTask(void);

#endif // _TM1638_MODES_H
