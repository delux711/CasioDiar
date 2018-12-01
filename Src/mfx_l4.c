#include "mfx_l4.h"

void mfx_init(void) {
    uint8_t i = 0u;
    HI2Cmfx_vInitPort();

    i = 0x83u;
    if(false == HI2Cmfx_bSetAddr(i)) {
        i = 0x84u;
        if(true == HI2Cmfx_bSetAddr(i)) {
            i = 5u;
        }
    }
    (void)i;
}
