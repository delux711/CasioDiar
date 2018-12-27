#ifndef _MFX_L4_H
#define _MFX_L4_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "mfx_l4_i2c.h"
#include "timerLib.h"

#define MFX_DEFINE_COPY_PINS

typedef enum _e_mfx_status {
    MFX_STATUS_NOT_INIT,
    MFX_STATUS_INIT_WAKEUP,
    MFX_STATUS_INIT_REG_0x41,
    MFX_STATUS_INIT_REG_0x42,
    MFX_STATUS_INIT_REG_IDD_EN,
    MFX_STATUS_INIT_REG_SET_CONST_VALUE,
    MFX_STATUS_INIT_INT_EN,
    MFX_STATUS_INIT,
    MFX_STATUS_SLEEP,
    MFX_STATUS_MEASUREMENT,
    MFX_STATUS_DONE,
    MFX_STATUS_ERROR
} e_mfx_status;

extern e_mfx_status mfx_handleTask(void);
extern int32_t mfx_getData(void);
extern void mfx_convertToChar(uint8_t *buff, int32_t value);
extern void mfx_init(void);
extern bool mfx_initForced(void);
extern void mfx_iddReqMeas(uint8_t predelay);
extern int32_t mfx_iddGetMeas(void);

#endif
