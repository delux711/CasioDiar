#ifndef _MFX_L4_H
#define _MFX_L4_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mfx_l4_i2c.h"

typedef enum _e_mfx_status {
    MFX_STATUS_NOT_INIT,
    MFX_STATUS_INIT_REG_0x41,
    MFX_STATUS_INIT_REG_0x42,
    MFX_STATUS_INIT_REG_IDD_EN,
    MFX_STATUS_INIT_REG_SET_CONST_VALUE,
    MFX_STATUS_INIT_REG2,
    MFX_STATUS_INIT_REG3,
    MFX_STATUS_INIT,
    MFX_STATUS_ERROR,
    MFX_STATUS_SLEEP
} e_mfx_status;

extern e_mfx_status mfx_handleTask(void);
void mfx_init(void);
bool mfx_initForced(void);

#endif
