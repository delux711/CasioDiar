#ifndef __casioDiar_H
#define __casioDiar_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

typedef enum _cd_state_e {
    CD_STATE_NOT_INIT,
    CD_STATE_SLEEP,
    CD_STATE_SENDING,
    CD_STATE_ERROR,
    CD_STATE_RECEIVING
} cd_state_e;


extern cd_state_e CD_task(void);
extern uint8_t CD_receive(void);
extern void CD_sendToDiarConst(uint8_t *buff);
extern void CD_senToDiarEndCommunication(void);

#endif
