#include "casioDiar.h"
#include "serialPort.h"
#include "serialPort_u1.h"
#include "timerLib.h"
#include "getJoystick.h"
#include <stdio.h>

#define CD_TIMEOUT_DEF    (1000000u)
#define CD_MAX_NOT_ANSWERE  (5u)

/*
typedef enum _cd_waitChar_e {
    CD_WAIT_CHAR_WAITING,
    CD_WAIT_CHAR_RECEIVED,
    CD_WAIT_CHAR_ERROR
} cd_waitChar;
*/

typedef enum _cd_state_e {
    CD_STATE_NOT_INIT,
    CD_STATE_SLEEP,
    CD_STATE_SENDING,
    CD_STATE_RECEIVING
} cd_state_e;

typedef enum _cd_state_send_e {
    CD_STATE_SENDING_SLEEP,
    CD_STATE_SENDING_CR,        // 0x0D
    CD_STATE_SENDING_LF,        // 0x0A
    CD_STATE_SENDING_COLON    // dvojbodka
} cd_state_sending_e;

typedef enum _cd_state_receiving_e {
    CD_STATE_RECEIVING_SLEEP,
    CD_STATE_RECEIVING_CR,
    CD_STATE_RECEIVING_WAIT_CR,
    CD_STATE_RECEIVING_LF,
    CD_STATE_RECEIVING_WAIT_FOR_0X11,
    CD_STATE_RECEIVING_0X11,    // white for 0x11 from diar
    CD_STATE_RECEIVING_DATA1_SEND,
    CD_STATE_RECEIVING_DATA_WAIT,
    CD_STATE_RECEIVING_NEW_NOTE,
    CD_STATE_RECEIVING_NEW_NOTE_WAIT,
    CD_STATE_RECEIVING_SEND_DATA,
    CD_STATE_RECEIVING_SEND_DATA_WAIT,
    CD_STATE_RECEIVING_SAVE,
    CD_STATE_RECEIVING_SAVE_WAIT,
    CD_STATE_RECEIVING_WAIT_TO_END,
    CD_STATE_RECEIVING_END_COMMUNICATION
} cd_state_receiving_e;

static cd_state_e cd_state = CD_STATE_NOT_INIT;
static cd_state_receiving_e cd_state_receive = CD_STATE_RECEIVING_CR;
static cd_state_sending_e cd_state_send = CD_STATE_SENDING_CR;

static bool cd_receiving = false;
static uint16_t cd_toPrepareOffset;
static uint32_t cd_timeout = CD_TIMEOUT_DEF;
static uint8_t cd_buffer[300];
static uint16_t cd_buffPointer = 0u;
static uint8_t *cd_toSendPointer;
static uint8_t cd_buffToSend1[] = ":02000002A0005C:0580000041686F6A31C8:00000001FF";
static uint8_t cd_buffToSendNewNote[] = ":02000002A0005C";
static uint8_t cd_buffToSendBuff[300] = ":0580000041686F6A31C8"; // Ahoj1
//atic uint8_t cd_buffToSendBuff[300] = ":0580xx0041686F6A31C8"; // Ahoj1 xx offset
//static uint8_t cd_buffToSendAhoj[] = "12345678901234567890";
static uint8_t cd_buffToSendAhoj[] = "1111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFF1111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000aaaaaaaaaabbbbbbbbbbccccccccccddddddddddeeeeeeeeeeffffffffff0000000000111111111122222222223333333333444444444455555555556666";
//1111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000AAAAAAAAAABBBBBBBBBBCCCCCCCCCCDDDDDDDDDDEEEEEEEEEEFFFFFFFFFF1111111111222222222233333333334444444444555555555566666666667777777777888888888899999999990000000000aaaaaaaaaabbbbbbbbbbccccccccccddddddddddeeeeeeeeeeffffffffff0000000000111111111122222222223333333333444444444455555555556666

static uint8_t cd_buffToSendSave[] = ":00000001FF";
static uint8_t cd_buffToSendEndCom[] = ":000000FF01";
static uint8_t cd_countNotAns = 0u;

void CD_task_send(void);
bool CD_task_receive(void);
bool bWaitForCharacter(uint8_t ch);
bool bIsReceivedCharacter(uint8_t ch);
void CD_sendBuff(uint8_t *buff, cd_state_receiving_e newState, uint16_t setTim);
void CD_CommandTimer(cd_state_receiving_e finalState, cd_state_receiving_e waitState);
uint8_t CD_sendPrepareNote(uint8_t *buffText, uint16_t offset);
void CD_changeToCdFormat(uint8_t *buff, uint8_t ch);

void CD_buffToSendClear(void);
void CD_buffToSendAdd(uint8_t ch);
void CD_buffToSendAddCdFormat(uint8_t ch);
uint8_t CD_buffToSendCRC(void);
uint8_t CD_buffToSendCRC(void);
uint8_t CD_buffToValue(uint8_t chHi, uint8_t chLo);

bool cd_test = false;

bool CD_task(void) {
    bool temp = false;
    if(cd_test != false) {    
        if(TIM_delayIsTimerDown(DELAY_TIMER_TEST) == true) {
            HDIO_testPinOff();
            cd_test = false;
        }
    }

    if(cd_state == CD_STATE_SLEEP) {

    } else if(cd_state == CD_STATE_RECEIVING) {
        temp = CD_task_receive();
    } else if(cd_state == CD_STATE_SENDING) {
        CD_task_send();
    }else if(cd_state == CD_STATE_NOT_INIT) {
        cd_state = CD_STATE_SLEEP;
        cd_state_receive = CD_STATE_RECEIVING_SLEEP;
        
        SP_init();
        TIM_delayInit();
        SPu1_init();

        HDIO_testPinInit();
        HDIO_testPinOff();
        TIM_delaySetTimer(DELAY_TIMER_TEST, 3u);
        //cd_test = true;
    }
    return temp;
}        

void CD_task_send(void) {
    if(cd_state_send == CD_STATE_SENDING_SLEEP) {
        
    } else if(cd_state_send == CD_STATE_SENDING_CR) {
        if(bWaitForCharacter('\r') == true) {
            cd_state_send = CD_STATE_SENDING_LF;
        }
    } else if(cd_state_send == CD_STATE_SENDING_LF) {
        if(bWaitForCharacter('\n') == true) {
            cd_state_send = CD_STATE_SENDING_COLON;
        }
    } else if(cd_state_send == CD_STATE_SENDING_COLON) {
        if(bWaitForCharacter('\n') == true) {
            cd_state_send = CD_STATE_SENDING_SLEEP;
            cd_state = CD_STATE_SLEEP;
        }
    }
}

bool CD_task_receive(void) {
    uint8_t size;
    cd_receiving = true;
    switch(cd_state_receive) {
        case CD_STATE_RECEIVING_SLEEP: break;
        case CD_STATE_RECEIVING_CR:
            if(cd_countNotAns < CD_MAX_NOT_ANSWERE) {
                HDIO_testPinOn();
                SPu1_sendChar(0x0Du);    // 0x0D = \r
                HDIO_testPinOff();
                cd_state_receive = CD_STATE_RECEIVING_WAIT_CR;
                TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, 5u);
            } else {
                cd_countNotAns = 0u;
                cd_state_receive = CD_STATE_RECEIVING_SLEEP;
                cd_state = CD_STATE_SLEEP;
            }
            break;
        case CD_STATE_RECEIVING_WAIT_CR:
            if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                cd_state_receive = CD_STATE_RECEIVING_LF;
            }
            break;
        case CD_STATE_RECEIVING_LF:
            GPIOD->ODR &= (~GPIO_ODR_OD0_Msk); // pin OFF
            SPu1_sendChar(0x0Au);    // 0x0A = \n
            cd_state_receive = CD_STATE_RECEIVING_WAIT_FOR_0X11;
            TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, 50u);
            break;
        case CD_STATE_RECEIVING_WAIT_FOR_0X11:
            if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                cd_countNotAns++;
                cd_state_receive = CD_STATE_RECEIVING_CR;
            } else {
                if(bIsReceivedCharacter(0x11u) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_NEW_NOTE;
                }
            }
            break;
        case CD_STATE_RECEIVING_DATA1_SEND:
                CD_sendBuff(cd_buffToSend1, CD_STATE_RECEIVING_DATA_WAIT, 400u); break;
        case CD_STATE_RECEIVING_DATA_WAIT:
                if(bIsReceivedCharacter(0x23u) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_SLEEP;
                }
                if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_SLEEP;
                }
                break;
        case CD_STATE_RECEIVING_END_COMMUNICATION:
                CD_sendBuff(cd_buffToSendEndCom, CD_STATE_RECEIVING_SLEEP, 0u);
                cd_state = CD_STATE_SLEEP;
                break;
        case CD_STATE_RECEIVING_NEW_NOTE:
                CD_sendBuff(cd_buffToSendNewNote, CD_STATE_RECEIVING_NEW_NOTE_WAIT, 100u); break;
        case CD_STATE_RECEIVING_NEW_NOTE_WAIT:
                if(bIsReceivedCharacter(0x23u) == true) {
                    cd_toPrepareOffset = 0u;
                    cd_state_receive = CD_STATE_RECEIVING_SEND_DATA;
                } else if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_END_COMMUNICATION;
                }
                break;
        case CD_STATE_RECEIVING_SEND_DATA:
                size = CD_sendPrepareNote(cd_toSendPointer, cd_toPrepareOffset);
                if(size < 0x80u) {
                    size = 0u;
                    CD_sendBuff(cd_buffToSendBuff, CD_STATE_RECEIVING_SAVE, 0u);
                } else {
                    cd_toPrepareOffset += size;
                    CD_sendBuff(cd_buffToSendBuff, CD_STATE_RECEIVING_SEND_DATA_WAIT, 50u);
                }
                break;
        case CD_STATE_RECEIVING_SEND_DATA_WAIT:
                if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_SEND_DATA;
                }
                break;
        case CD_STATE_RECEIVING_SAVE:
                CD_sendBuff(cd_buffToSendSave, CD_STATE_RECEIVING_SAVE_WAIT, 1000u); break;
        case CD_STATE_RECEIVING_SAVE_WAIT:
                cd_receiving = false;
                if(SPu1_isNewData() == true) {
                    switch(SPu1_getData()) {
                        case 0x23u:
                            cd_state_receive = CD_STATE_RECEIVING_WAIT_TO_END;
                            TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, 5000u);
                            break;
                        case 0x21u: // memory full
                            cd_state_receive = CD_STATE_RECEIVING_END_COMMUNICATION;
                            break;
                        default:
                            if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                                cd_state_receive = CD_STATE_RECEIVING_END_COMMUNICATION;
                            }
                            break;
                    }
                }/*
                if(bIsReceivedCharacter(0x23u) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_WAIT_TO_END;
                    TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, 5000u);
                } else if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_END_COMMUNICATION;
                }*/
                break;
        case CD_STATE_RECEIVING_WAIT_TO_END:
                if(TIM_delayIsTimerDown(DELAY_TIMER_CASIO_DIAR) == true) {
                    cd_state_receive = CD_STATE_RECEIVING_END_COMMUNICATION;
                }
                break;
        default: break;
    }
    return cd_receiving;
}

uint8_t CD_sendPrepareNote(uint8_t *buffText, uint16_t offset) {
    uint16_t i;
    uint8_t j, size;
    CD_buffToSendClear();
    // START character :
    CD_buffToSendAdd(':');
    // couting size to send
    size = 0u;
    while((buffText[size + offset] != '\0') && (size < 0x80u)) {    // 0x80 = 128 bytov dokazem naraz poslat
        size++;
    }
    CD_buffToSendAddCdFormat(size);
    CD_buffToSendAdd('8');
    if(offset < 0x100u) {
        CD_buffToSendAdd('0');
    } else {
        CD_buffToSendAdd('1');
    }
    //CD_buffToSendAdd('0');    // offset in hex npr 0
    //CD_buffToSendAdd('0');    // offset            F - 0F=15 bytov od zaciatku
    CD_buffToSendAddCdFormat(offset);
    CD_buffToSendAdd('0');
    CD_buffToSendAdd('0');
    i = offset;
    j = size;
    if(size > 0x80u) {
        j = 0x80u;
    }
    while(j != 0u) {
        CD_buffToSendAddCdFormat(buffText[i++]);
        j--;
    }
    CD_buffToSendAddCdFormat(CD_buffToSendCRC());
    CD_buffToSendAdd('\0');
    return size;
}

// cd_buffToSendBuff[50] = ":0580000041686F6A31C8"; // Ahoj1
//static uint8_t cd_buffToSendAhoj[] = "Ahoj1";
uint8_t CD_buffToSendCRC(void) {
    uint8_t ch1, ch2, crc;
    uint16_t i, count;
    i = 1u;
    ch1 = cd_buffToSendBuff[i++];
    ch2 = cd_buffToSendBuff[i++];
    crc = CD_buffToValue(ch1, ch2);
    for(count = (3u + crc); count != 0; count--) {
        ch1 = cd_buffToSendBuff[i++];
        ch2 = cd_buffToSendBuff[i++];
        crc = (uint8_t)(0xFFu & (crc + CD_buffToValue(ch1, ch2)));
    }
    return (uint8_t) (0x100 - crc);
}

uint8_t CD_buffToValue(uint8_t chHi, uint8_t chLo) {
    uint8_t i, ch, temp;
    temp = 0u;
    ch = chHi;
    for(i = 0u; i < 2u; i++) {
        temp <<= 4u;
        if(ch <= '9') {
            temp |= (ch - '0');
        } else {
            temp |= (ch - ('0' + 7u));   // A-F
        }
        ch = chLo;
    }
    return temp;
}

void CD_buffToSendClear(void) {
    cd_buffPointer = 0u;
}

void CD_buffToSendAdd(uint8_t ch) {
    if(cd_buffPointer < (uint16_t) sizeof(cd_buffToSendBuff)) {
        cd_buffToSendBuff[cd_buffPointer++] = ch;
    }
}

void CD_buffToSendAddCdFormat(uint8_t ch) {
    uint8_t temp;
    temp = (ch >> 4u) + '0';
    if(temp > '9') {
        temp += 7u; // ascii A-F
    }
    CD_buffToSendAdd(temp);
    temp = (0x0Fu & ch) + '0';
    if(temp > '9') {
        temp += 7u; // ascii A-F
    }
    CD_buffToSendAdd(temp);
}

void CD_sendBuff(uint8_t *buff, cd_state_receiving_e newState, uint16_t setTim) {
    uint8_t ch;
    uint16_t i;
    i = 0u;
    ch = buff[0u];
    while(ch != '\0') {
        if(bIsReceivedCharacter(0x13u) == true) {   // May send 0x13 (ASCII 19/Ctrl-S/XOFF) to request a temporary halt (too much data too fast for it)
            SPu1_pauseOn();
            while(bIsReceivedCharacter(0x11u) == false);
            SPu1_pauseOff();
        }
        ch = buff[i++];
        SPu1_sendChar(ch);
    }
    cd_state_receive = newState;
    TIM_delaySetTimer(DELAY_TIMER_CASIO_DIAR, setTim);
}

uint8_t CD_receive(void) {
    cd_state = CD_STATE_SENDING;
    cd_state_send = CD_STATE_SENDING_CR;
    return cd_buffer[0];
}

void CD_sendToDiarConst(uint8_t *buff) {
    if(buff == 0u) {
        cd_toSendPointer = cd_buffToSendAhoj;
    } else {
        cd_toSendPointer = buff;
    }
    CD_CommandTimer(CD_STATE_RECEIVING_CR, CD_STATE_RECEIVING_SLEEP);
    CD_CommandTimer(CD_STATE_RECEIVING_CR, CD_STATE_RECEIVING_WAIT_TO_END);
}

void CD_senToDiarEndCommunication(void) {
    CD_CommandTimer(CD_STATE_RECEIVING_END_COMMUNICATION, CD_STATE_RECEIVING_WAIT_TO_END);
}

void CD_CommandTimer(cd_state_receiving_e finalState, cd_state_receiving_e waitState) {
    if(TIM_delayIsTimerDown(DELAY_TL) == true) {
        if(cd_state_receive == waitState) {
            cd_countNotAns = 0u;
            TIM_delaySetTimer(DELAY_TL, 500);
            cd_state_receive = finalState;
            cd_state = CD_STATE_RECEIVING;
        }
    }
}

bool bWaitForCharacter(uint8_t ch) {
    bool bReturn;
    bReturn = false;
    if(cd_timeout == 0) {
        cd_state = CD_STATE_SLEEP;
        cd_timeout = CD_TIMEOUT_DEF;
    } else {
        cd_timeout--;
        if(SP_isNewData() == true) {
            if(SP_getData() == ch) {
                cd_timeout = CD_TIMEOUT_DEF;
                bReturn = true;
            }
        }
    }
    return bReturn;
}

bool bIsReceivedCharacter(uint8_t ch) {
    bool ret;
    ret = false;
    if(SPu1_isNewData() == true) {
        if(SPu1_getData() == ch) {
            ret = true;
        }
    }
    return ret;
}
