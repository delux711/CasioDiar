#include "tm1638_modes.h"

static bool tmm_show = false;
static uint8_t tmm_tlBuff[9];
static uint8_t tmm_chLast = 0u;
static uint8_t tmm_tlLast = 0u;
static TMM_tStatus tmm_status = TMM_STATUS_NOT_INIT;

void tmm_changeModeInit(TMM_tStatus status);
void tmm_changeModeIf(void);
bool tmm_blink(bool digit, bool led, uint8_t button);
void tmm_tlBuff0plus(void);

void tmm_changeModeInit(TMM_tStatus status) {
    uint8_t i;
    //uint8_t tmm_tlBuff[9] = {0xFF, '0', '0', '0', '0', '0', '0', '0', '0'};
    //tmm_tlBuff[] = {0xFF, (uint8_t)('0' | 0x80u), '0', '0', '0', '0', '0', '0', (uint8_t)('0' | 0x80u)};
    tmm_status = status;
    tmm_tlLast = 0u;
    switch(tmm_status) {
        case TMM_STATUS_MODE_3:
        case TMM_STATUS_MODE_2:
            tmm_tlBuff[0] = 1u;
            for(i = 1u; i < 9u; i++) {
                tmm_tlBuff[i] = '-';
            }
            tm1638_showLed(0u);
            break;
        case TMM_STATUS_MODE_5_LED_PULSE:
            tmm_chLast = 0u;
            for(i = 1u; i < 9u; i++) {
                tmm_tlBuff[i] = '8';
            }
            tm1638_showLed(0xFFu);
            break;
        case TMM_STATUS_MODE_4_TEMP:
        case TMM_STATUS_MODE_0:
            tmm_tlBuff[0] = 0xFFu;
            for(i = 1u; i < 9u; i++) {
                tmm_tlBuff[i] = ' ';
            }
            tm1638_showLed(0u);
            break;
        case TMM_STATUS_MODE_1:
        default:
            for(i = 1u; i < 9u; i++) {
                tmm_tlBuff[i] = '1';
            }
            tmm_tlBuff[0] = 0xFFu;
            tmm_tlBuff[1] = '8';
            tmm_tlBuff[2] = '0';
            tmm_tlBuff[7] = '0';
            tmm_tlBuff[8] = '8';
            tm1638_showLed(0xFFu);
            break;
    }
    tm1638_show(&tmm_tlBuff[1]);
}

void tmm_changeModeIf(void) {
    uint8_t i, j;
    if(('8' == tmm_tlBuff[1]) && ('8' == tmm_tlBuff[8])) {
        j = 0u;
        for(i = 3u; i < 8u; i++) {
            if('1' == tmm_tlBuff[i]) {
                j++;
            } else {
                break;
            }
        }
        if(5u == j) {
            j = tmm_tlBuff[2];
            if(j == '0') {
                tmm_changeModeInit(TMM_STATUS_MODE_0);
            } else if(j < '4') {
                tmm_changeModeInit((TMM_tStatus)(j - '0'));
            } else if(j == '4') {
                tmm_changeModeInit(TMM_STATUS_MODE_4_TEMP);
            } else if(j == '5') {
                tmm_changeModeInit(TMM_STATUS_MODE_5_LED_PULSE);
            }
        }
    }
}

bool tmm_blink(bool digit, bool led, uint8_t button) {
    bool ret;
    if(0u != TIM_delayIsTimerDown(DELAY_TM1638_BLINK)) {
        TIM_delaySetTimer(DELAY_TM1638_BLINK, 500u);
        if(0u == (0x80u & tmm_tlBuff[0])) {
            if(0u != digit)
                tm1638_showPos(tmm_tlBuff[0], tmm_tlBuff[tmm_tlBuff[0]]);
            if(0u != led)
                tm1638_showLed(0u);
            tmm_tlBuff[0] |= 0x80u;
        } else {
            tmm_tlBuff[0] &= 0x7Fu;
            if(0u != digit)
                tm1638_showPos(tmm_tlBuff[0], ' ');
            if(0u != led)
                tm1638_showLed(0x80u >> (tmm_tlBuff[0] - 1u));
        }
    }
    if((0u != tmm_tlLast) && (0u == button)) {
        ret = true;
        tmm_tlLast = 0u;
    } else {
        ret = false;
        tmm_tlLast = button;
    }
    return ret;
}

void tmm_tlBuff0plus(void) {
    tmm_tlBuff[0]++;
    if(8u < (0x7F & tmm_tlBuff[0])) {
        tmm_tlBuff[0] &= 0xF0u;
        tmm_tlBuff[0] |= 0x01u;
    }
}

TMM_tStatus TMM_getState(void) {
    return tmm_status;
}

void TMM_handleTask(void) {
    uint8_t i, ch;
    uint8_t *p;
    if(TM1638_STATUS_TL_DONE == TM1638_handleTaskTl()) {
        if((TIM_delayIsTimerDown(DELAY_TM1638) == true) && (0u != tmm_show)) {
            tmm_show = false;
            ch = tm1638_getTl();
            TIM_delaySetTimer(DELAY_TM1638, 100u);

            switch(tmm_status) {
                case TMM_STATUS_MODE_4_TEMP:
                case TMM_STATUS_MODE_0:
                    if(0u != ch) {
                        tmm_changeModeInit(TMM_STATUS_MODE_1);
                    }
                    break;
                case TMM_STATUS_MODE_1:
                    if(0u != ch) {
                        tm1638_showLed(ch);
                        for(i = 1u; i < 9u; i++) {
                            if(ch & 0x80u) {
                                tmm_tlBuff[i]++;
                                if(('9' | 0x80u) < tmm_tlBuff[i]) {
                                    tmm_tlBuff[i] = '0';
                                } else if(('9' < tmm_tlBuff[i]) && (0u == (tmm_tlBuff[i] & 0x80u))) {
                                    tmm_tlBuff[i] = '0' | 0x80u;
                                }
                            }
                            ch <<= 1u;
                        }
                        tm1638_show(&tmm_tlBuff[1]);
                        tmm_changeModeIf();
                    } else {
                        tm1638_showLed(0u);
                    }
                    break;

                case TMM_STATUS_MODE_5_LED_PULSE:
                    if(0u == ch) {
                        if(0u != TIM_delayIsTimerDown(DELAY_TM1638_BLINK)) {
                            TIM_delaySetTimer(DELAY_TM1638_BLINK, 500u);
                            if(0u == tmm_tlBuff[0]) {
                                tmm_tlBuff[0] = 0xFFu;
                            } else {
                                tmm_tlBuff[0] = 0u;
                            }
                            tm1638_showLed(tmm_tlBuff[0]);      // blink all leds
                        }
                        break;
                    }
                    tmm_status = TMM_STATUS_MODE_5_LED_PULSE_WAIT;  // no break!
                case TMM_STATUS_MODE_5_LED_PULSE_CHECK:
                
                    for(i = 0u; i < 8u; i++) {
                        if(0u != (0x80u & ch)) {
                            break;
                        }
                        ch <<= 1u;
                    }
                    if(i != tmm_chLast) {
                        tmm_chLast = i;
                        tmm_tlBuff[0] = i;
                        tm1638_showLed(i);
                        tmm_status = TMM_STATUS_MODE_5_LED_PULSE_WAIT;
                    } else {
                        if(0u != tm1638_changePulse(i)) {
                            tmm_changeModeInit(TMM_STATUS_MODE_4_TEMP);
                            tmm_chLast = 0u;
                        }
                    }
                    break;
                case TMM_STATUS_MODE_5_LED_PULSE_WAIT:
                    if(0u != tmm_blink(false, true, ch)) {
                        tmm_status = TMM_STATUS_MODE_5_LED_PULSE_CHECK;
                    }
                    break;

                case TMM_STATUS_MODE_2:
                    if(0u != tmm_blink(true, true, ch)) {
                        for(i = 0u; i < 8u; i++) {
                            if(0u != (ch & 0x80u)) {
                                i++;
                                tmm_tlBuff[0] &= 0x7Fu;
                                tmm_tlBuff[tmm_tlBuff[0]] = (i + '0');
                                tmm_tlBuff[0]++;
                                if(8u < tmm_tlBuff[0]) {
                                    tmm_tlBuff[0] = 1u;
                                }
                                tm1638_show(&tmm_tlBuff[1]);
                                break;
                            }
                            ch <<= 1u;
                        }
                        tmm_changeModeIf();
                    }
                    break;

                case TMM_STATUS_MODE_3:
                    if(0u != tmm_blink(false, true, ch)) {   // only LED blink
                        if(0u != (0x80u & ch)) {
                            tmm_tlBuff[0]--;
                            if((0x0Fu & tmm_tlBuff[0]) < 1u) {
                                tmm_tlBuff[0] |= 0x08u;
                            }
                        }
                        if(0u != (0x40u & ch)) {
                            tmm_tlBuff0plus();
                        }
                        if(0u != (0x20u & ch)) {
                            tmm_status = TMM_STATUS_MODE_3_NUM;
                        }
                        tmm_tlBuff[0] |= 0x80u; // aby vo funckii tmm_blink zasvietila ledka po dtlaceni tl.
                        TIM_delaySetTimer(DELAY_TM1638_BLINK, 0u);
                    }
                    break;
                case TMM_STATUS_MODE_3_NUM:
                    if(0u != ch) {
                        p = &tmm_tlBuff[(0x7Fu & tmm_tlBuff[0])];
                        if(0u != (0x80u & ch)) {
                            *p = *p - 1u;
                            if(*p < '0') {
                                *p = '9';
                            }
                        }
                        if(0u != (0x40u & ch)) {
                            *p = *p + 1u;
                            if(('9' < *p) || (('-' + 1u) == *p)) {
                                *p = '0';
                            }
                        }
                        if(0u != (0x20u & ch)) {
                            tmm_tlBuff0plus();
                            tmm_status = TMM_STATUS_MODE_3;
                        }
                        tmm_tlBuff[0] &= 0x7Fu; // aby vo funckii tmm_blink zasvietila cislica po dtlaceni tl.
                        TIM_delaySetTimer(DELAY_TM1638_BLINK, 0u);
                    }
                    (void)tmm_blink(true, false, ch); // only number blink
                    if(TMM_STATUS_MODE_3_NUM != tmm_status) {
                        tm1638_show(&tmm_tlBuff[1]);
                        tmm_changeModeIf();
                    }
                    break;

                case TMM_STATUS_NOT_INIT:
                default:
                    tm1638_init();
                    tm1638_show((uint8_t*)"98765432");
                    //tm1638_showPos(4u, '0');
                    tmm_changeModeInit(TMM_STATUS_MODE_4_TEMP);
                    break;
            } // switch
        }
    } else {
        tmm_show = true;
    }
}
