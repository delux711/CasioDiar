#ifndef __casioDiar_H
#define __casioDiar_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

extern void CD_task(void);
extern uint8_t CD_receive(void);
extern void CD_sendToDiarConst(uint8_t *buff);
extern void CD_senToDiarEndCommunication(void);

#endif
