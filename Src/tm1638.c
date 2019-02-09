#include "tm1638.h"

static bool tm_isDataToSend = false;
static bool tm_isPositionToSend = false;
static uint8_t tm_savePosition;
static uint8_t tm_saveChar;
static uint8_t tm_data[16];
static TM1638_status_tl statusTl = TM1638_STATUS_TL_NOT_INIT;
static uint8_t statusTlCount;
static uint8_t statusTlBuff[4];
static uint8_t statusTlMsk;

void tm1638_initPort(void);
void tm1638_sendCommand(uint8_t comm);
void tm1638_sendData(uint8_t data);
void tm1638_setAddress(uint8_t address, bool stop);
uint8_t tm_convToDigit(uint8_t ch);
void tm_sendBuffToShow(void);
void tm1638_showToSend(bool isDifferent);

#define TM_STB_0_MAP(PORT,PIN)      do { GPIO##PORT->BSRR |= GPIO_BSRR_BR##PIN; } while(0u)
#define TM_STB_1_MAP(PORT,PIN)      do { GPIO##PORT->BSRR |= GPIO_BSRR_BS##PIN; } while(0u)
#define TM_STB_OUT_MAP(PORT,PIN)    do { GPIO##PORT->MODER &= ~GPIO_MODER_MODE##PIN##_Msk; \
                                       GPIO##PORT->MODER |= (1u << GPIO_MODER_MODE##PIN##_Pos); } while(0) // STB - output from uP

#define TM_CLK_0_MAP(PORT,PIN)      do { GPIO##PORT->BSRR |= GPIO_BSRR_BR##PIN; } while(0u)
#define TM_CLK_1_MAP(PORT,PIN)      do { GPIO##PORT->BSRR |= GPIO_BSRR_BS##PIN; } while(0u)
#define TM_CLK_OUT_MAP(PORT,PIN)    do { GPIO##PORT->MODER &= ~GPIO_MODER_MODE##PIN##_Msk; \
                                       GPIO##PORT->MODER |= (1u << GPIO_MODER_MODE##PIN##_Pos); } while(0)// CLK - output from uP

#define TM_DIO_CONFIG_MAP(PORT,PIN) do { GPIO##PORT->PUPDR &= ~(GPIO_PUPDR_PUPD##PIN##_Msk);      \
                                         GPIO##PORT->PUPDR |= (1u << GPIO_PUPDR_PUPD##PIN##_Pos); \
                                         GPIO##PORT->OTYPER |= GPIO_OTYPER_OT##PIN; } while(0) // DIO-PULL UP and OPEN collector
#define TM_DIO_IN_MAP(PORT,PIN)     do { GPIO##PORT->MODER &= ~GPIO_MODER_MODE##PIN##_Msk; } while(0u)
#define TM_DIO_OUT_MAP(PORT,PIN)    do { GPIO##PORT->MODER |= (1u << GPIO_MODER_MODE##PIN##_Pos); } while(0u)
#define TM_DIO_DATA_MAP(PORT,PIN)   (0u != (GPIO##PORT->IDR & GPIO_IDR_ID##PIN))
#define TM_DIO_0_MAP(PORT,PIN)      do { GPIO##PORT->BSRR |= GPIO_BSRR_BR##PIN; } while(0u)
#define TM_DIO_1_MAP(PORT,PIN)      do { GPIO##PORT->BSRR |= GPIO_BSRR_BS##PIN; } while(0u)


uint8_t tm_convToDigit(uint8_t ch) {
    uint8_t ret;
    if(255u != ch) { // 255 is '°' character
        if(0u != (0x80u & ch)) {
            ch &= 0x7Fu;
            ret = 0x80u;
        } else {
            ret = 0u;
        }
        if(ch & 0x40u) {
            ch &= 0xDF;
        }
        switch(ch) {
            case '0': ret |= 0x3Fu; break;
            case '1': ret |= 0x06u; break;
            case 'Z':
            case '2': ret |= 0x5Bu; break;
            case '3': ret |= 0x4Fu; break;
            case '4': ret |= 0x66u; break;
            case 'S':
            case '5': ret |= 0x6Du; break;
            case '6': ret |= 0x7Du; break;
            case '7': ret |= 0x07u; break;
            case '8': ret |= 0x7Fu; break;
            case 'G':
            case '9': ret |= 0x6Fu; break;
            case '-': ret |= 0x40u; break;
            case '.':
            case ',': ret |= 0x80u; break;
            case ' ': ret |= 0x00u; break;
            case 'A': ret |= 0x77u; break;
            case 'B': ret |= 0x7Cu; break;
            case 'C': ret |= 0x39u; break;
            case 'D': ret |= 0x5Eu; break;
            case 'E': ret |= 0x79u; break;
            case 'F': ret |= 0x71u; break;
            case 'H': ret |= 0x76u; break;
            case 'I': ret |= 0x04u; break;
            case 'J': ret |= 0x1Fu; break;
            case 'L': ret |= 0x38u; break;
            case 'N': ret |= 0x54u; break;
            case 'O': ret |= 0x5Cu; break;
            case 'P': ret |= 0x73u; break;
            case 'R': ret |= 0x50u; break;
            case 'T': ret |= 0x78u; break;
            case 'U': ret |= 0x3Eu; break;
            case 'Y': ret |= 0x6Eu; break;
            case 'c': ret |= 0x58u; break;
        //    case : ret = 0xu; break;
            default: ret = 0x80u; break;
        }
    } else {
        ret = 0x63u; // '°'
    }
    return ret;
}

void tm1638_showLed(uint8_t led) {
    bool isDifferent;
    uint8_t i, j, temp;
    isDifferent = false;
    j = 1u;
    for(i = 0u; i < 8u; i++) {
        if(0u != (0x80u & led)) {
            temp = 0x01u;
        } else {
            temp = 0u;
        }
        if(temp != tm_data[j]) {
            tm_data[j] = temp;
            isDifferent = true;
        }
        j += 2u;
        led <<= 1u;
    }
    tm1638_showToSend(isDifferent);
}

void tm1638_show(uint8_t *buff) {
    bool isDifferent;
    uint8_t i, j, ch, size;
    isDifferent = false;
    j = 0u;
    size = 8u;
    for(i = 0u; i < size; i++) {
        ch = tm_convToDigit(buff[i]);
        if('.' == buff[i+1]) {
            ch |= 0x80u;              // if is next char '.' adding '.' on digit
            i++;
            size++;
        }
        if(ch != tm_data[j]) {
            tm_data[j] = ch;
            j += 2u;
            isDifferent = true;
        } else {
            j += 2u;
        }
    }
    tm1638_showToSend(isDifferent);
}

void tm1638_showToSend(bool isDifferent) {
    if(true == isDifferent) {
        if(statusTl == TM1638_STATUS_TL_DONE) {
            tm_sendBuffToShow();
        } else {
            tm_isDataToSend = true;
        }
    }
}

void tm_sendBuffToShow(void) {
    uint8_t i;
    tm1638_setAddress(0u, false);
    for(i = 0u; i < 16u; i++) {
        tm1638_sendData(tm_data[i]);
    }
    TM_STB_1();
}

void tm1638_showPos(uint8_t position, uint8_t ch) {
    if(TM1638_STATUS_TL_DONE == statusTl) {
        position = (position  * 2u) - 2u;
        ch = tm_convToDigit(ch);
        if(tm_data[position] != ch) {
            tm_data[position] = ch;
            tm1638_setAddress(position, false);
            tm1638_sendData(ch);
            TM_STB_1();
        }
    } else {
        tm_savePosition = position;
        tm_saveChar = ch;
        tm_isPositionToSend = true;
    }
}

void tm1638_setAddress(uint8_t address, bool stop) {
    if(address < 0x10u) {
        TM_STB_0();
        tm1638_sendData(TM1638_SET_ADDRESS + address);
        //tm1638_sendData(address);
        if(true == stop) {
            TM_STB_1();
        }
    }
}

void tm1638_sendCommand(uint8_t comm) {
    TM_STB_0();
    tm1638_sendData(comm);
    TM_STB_1();
}

void tm1638_sendPacket(uint8_t *buff, uint8_t size) {
    uint8_t i;
    TM_STB_0();
    for(i = 0u; i < size; i++) {
        tm1638_sendData(buff[i]);
    }
    TM_STB_1();
}

void tm1638_sendData(uint8_t data) {
    uint8_t i;
    for(i = 0u; i < 8u; i++) {
        TM_CLK_0();
        if(0u != (data & 0x01u)) {
            TM_DIO_1();
        } else {
            TM_DIO_0();
        }
        TM_CLK_1();
        data >>= 1u;
    }
}

void tm1638_readTl(uint8_t *buff) {
    bool pin;
    uint8_t i, j, ch;
    TM_STB_0();
    tm1638_sendData(TM1638_COMMAND_READ_DATA);
    TM_DIO_IN();
    for(i = 0u; i < 4u; i++) {
        ch = 0u;
        for(j = 0u; j < 8u; j++) {
            TM_CLK_0();
            ch >>= 1u;
            pin = TM_DIO_DATA();
            TM_CLK_1();
            if(true == pin) {
                ch |= 0x80u;
            }
            __NOP();
        }
        buff[i] = ch;
    }
    TM_DIO_OUT();
    TM_STB_1();
    tm1638_sendCommand(TM1638_COMMAND_WRITE_DATA);
}

void tm1638_initPort(void) {
    TM_STB_OUT();
    TM_STB_1();

    TM_CLK_OUT();
    TM_CLK_1();

    TM_DIO_IN();
    TM_DIO_CONFIG();
    TM_DIO_OUT();                                    // DIO - input/output with open collector, pull up from uP
    TM_DIO_1();
}

void tm1638_init(void) {
    uint8_t i;
    statusTl = TM1638_STATUS_TL_NOT_INIT;
    statusTlCount = 0u;
    tm1638_initPort();
    
    tm1638_sendCommand(TM1638_COMMAND_WRITE_DATA);
    tm1638_sendCommand(TM1638_COMMAND_LCD_ON_PULSE_1_16);
    tm1638_setAddress(0u, false);
    tm1638_sendData(0x01);
    tm1638_sendData(0x01);
    tm1638_sendData(0x02);
    tm1638_sendData(0x02);
    tm1638_sendData(0x04);
    tm1638_sendData(0x04);
    tm1638_sendData(0x08);
    tm1638_sendData(0x08);
    tm1638_sendData(0x10);
    tm1638_sendData(0x10);
    tm1638_sendData(0x20);
    tm1638_sendData(0x20);
    tm1638_sendData(0x40);
    tm1638_sendData(0x40);
    tm1638_sendData(0x80);
    tm1638_sendData(0x80);
    for(i = 0u; i < 16u; i++) {
        tm1638_sendData(0xFFu);
    }
    for(i = 0u; i < 16u; i++) {
        tm1638_sendData(0x00u);
    }
    TM_STB_1();
}

uint8_t tm1638_getTl(void) {
    return statusTlMsk;
}

void tm1638_communication(bool turnOnOff) {
    if(false == turnOnOff) { // communication turn off
        while(TM1638_STATUS_TL_DONE != TM1638_handleTaskTl());
        statusTl = TM1638_STATUS_COMMUNICATION_OFF;
        TM_STB_1(); // do not communication tm1638
    } else {
        if(TM1638_STATUS_COMMUNICATION_OFF == statusTl) {
            statusTl = TM1638_STATUS_TL_DONE;
        }
    }
}

/*
84218421
12345678
 1
0100 0000
   2
0001 0000
      3
0000 0100
        4
0000 0001
5
1000 0000
  6
0010 0000
     7
0000 1000
       8
0000 0010*/
TM1638_status_tl TM1638_handleTaskTl(void) {
    uint8_t i, ch;
    bool pin;
    switch(statusTl) {
        case TM1638_STATUS_TL_DONE:
            if(statusTlCount < TM1638_MAX_DONE_COUNT) {
                statusTlCount++;
            } else {
                statusTlCount = 0u;
                statusTl = TM1638_STATUS_TL_WRITE;
            }
            if(true == tm_isDataToSend) {
                tm_isDataToSend = false;
                tm_sendBuffToShow();
            } else if(true == tm_isPositionToSend) {
                tm_isPositionToSend = false;
                tm1638_showPos(tm_savePosition, tm_saveChar);
            }
            break;
        case TM1638_STATUS_TL_WRITE:
            TM_STB_0();
            tm1638_sendData(TM1638_COMMAND_READ_DATA);
            TM_DIO_IN();
            statusTl = TM1638_STATUS_TL_READ1;
            break;
        case TM1638_STATUS_TL_READ1:
        case TM1638_STATUS_TL_READ2:
        case TM1638_STATUS_TL_READ3:
        case TM1638_STATUS_TL_READ4:
            ch = 0u;
            for(i = 0u; i < 8u; i++) {
                TM_CLK_0();
                ch >>= 1u;
                pin = TM_DIO_DATA();
                TM_CLK_1();
                if(true == pin) {
                    ch |= 0x80u;
                }
                __NOP();
            }
            statusTlBuff[(uint8_t)(statusTl - TM1638_STATUS_TL_READ1)] = ch;
            statusTl++;
            break;
        case TM1638_STATUS_TL_READ_DONE:
            TM_DIO_OUT();
            TM_STB_1();
            tm1638_sendCommand(TM1638_COMMAND_WRITE_DATA);
            statusTl = TM1638_STATUS_TL_CALC;
            break;
        case TM1638_STATUS_TL_CALC:
            statusTlMsk = 0u;
            if(statusTlBuff[0] & 0xF0u) statusTlMsk |= 0x08u;   // tl5
            if(statusTlBuff[0] & 0x0Fu) statusTlMsk |= 0x80u;   // tl1
            if(statusTlBuff[1] & 0xF0u) statusTlMsk |= 0x04u;   // tl6
            if(statusTlBuff[1] & 0x0Fu) statusTlMsk |= 0x40u;   // tl2
            if(statusTlBuff[2] & 0xF0u) statusTlMsk |= 0x02u;   // tl7
            if(statusTlBuff[2] & 0x0Fu) statusTlMsk |= 0x20u;   // tl3
            if(statusTlBuff[3] & 0xF0u) statusTlMsk |= 0x01u;   // tl8
            if(statusTlBuff[3] & 0x0Fu) statusTlMsk |= 0x10u;   // tl4
            statusTl = TM1638_STATUS_TL_DONE;
            break;
        case TM1638_STATUS_TL_NOT_INIT:
            statusTlCount = 0u;
            tm1638_initPort();
            tm1638_sendCommand(TM1638_COMMAND_WRITE_DATA);
            statusTl = TM1638_STATUS_TL_NOT_INIT2;
            break;
        case TM1638_STATUS_TL_NOT_INIT2:
            tm1638_sendCommand(TM1638_COMMAND_LCD_ON_PULSE_1_16);
            statusTl = TM1638_STATUS_TL_WRITE;
            break;
        case TM1638_STATUS_COMMUNICATION_OFF:
            break;
        default: break;
    }
    return statusTl;
}
