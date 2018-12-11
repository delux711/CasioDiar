#include "tm1638.h"

static bool tm_isDataToSend = false;
static bool tm_isPositionToSend = false;
static uint8_t tm_savePosition;
static uint8_t tm_saveChar;
static uint8_t tm_data[14];
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


#define TM_STB_0()	do { GPIOE->BSRR |= GPIO_BSRR_BR13; } while(0u)
#define TM_STB_1()	do { GPIOE->BSRR |= GPIO_BSRR_BS13; } while(0u)

#define TM_CLK_0()	do { GPIOE->BSRR |= GPIO_BSRR_BR14; } while(0u)
#define TM_CLK_1()	do { GPIOE->BSRR |= GPIO_BSRR_BS14; } while(0u)

#define TM_DIO_IN()		do { GPIOE->MODER &= ~GPIO_MODER_MODE15_Msk; } while(0u)
#define TM_DIO_OUT()	do { GPIOE->MODER |= (1u << GPIO_MODER_MODE15_Pos); } while(0u)
#define TM_DIO_DATA()   (GPIOE->IDR & GPIO_IDR_ID15)
#define TM_DIO_0()		do { GPIOE->BSRR |= GPIO_BSRR_BR15; } while(0u)
#define TM_DIO_1()		do { GPIOE->BSRR |= GPIO_BSRR_BS15; } while(0u)

uint8_t tm_convToDigit(uint8_t ch) {
	uint8_t ret;
    if(true == (0x80u & ch)) {
        ch &= 0x7Fu;
        ret = 0x80u;
    } else {
        ret = 0u;
    }
	switch(ch) {
		case '0': ret |= 0x3Fu; break;
		case '1': ret |= 0x06u; break;
		case '2': ret |= 0x5Bu; break;
		case '3': ret |= 0x4Fu; break;
		case '4': ret |= 0x66u; break;
		case '5': ret |= 0x6Du; break;
		case '6': ret |= 0x7Du; break;
		case '7': ret |= 0x07u; break;
		case '8': ret |= 0x7Fu; break;
		case '9': ret |= 0x6Fu; break;
		case '-': ret |= 0x40u; break;
        case '.':
        case ',': ret |= 0x80u; break;
        case ' ': ret = 0u;
	//	case : ret = 0xu; break;
		default: ret = 0x80u; break;
	}
	return ret;
}

void tm1638_show(uint8_t *buff) {
	uint8_t i, j;
	j = 0u;
	for(i = 0u; i < 8u; i++) {
		tm_data[j++] = tm_convToDigit(buff[i]);
		tm_data[j++] = 0x01;	// led on
	}
    if(statusTl == TM1638_STATUS_TL_DONE) {
        tm_sendBuffToShow();
    } else {
        tm_isDataToSend = true;
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
        tm_data[position] = ch;
        tm1638_setAddress(position, false);
        tm1638_sendData(ch);
        TM_STB_1();
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
		if(true == (data & 0x01u)) {
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
    statusTlCount = 0u;
	GPIOE->MODER &= ~GPIO_MODER_MODE13_Msk;
	GPIOE->MODER |= (1u << GPIO_MODER_MODE13_Pos);	// STB - output from uP
	TM_STB_1();

	GPIOE->MODER &= ~GPIO_MODER_MODE14_Msk;
	GPIOE->MODER |= (1u << GPIO_MODER_MODE14_Pos);	// CLK - output from uP
	TM_CLK_1();

	TM_DIO_IN();
	GPIOE->PUPDR &= ~(GPIO_PUPDR_PUPD15_Msk);
	GPIOE->PUPDR |= (1u << GPIO_PUPDR_PUPD15_Pos);
	GPIOE->OTYPER |= GPIO_OTYPER_OT15;
	TM_DIO_OUT();									// DIO - input/output with open collector, pull up from uP
	TM_DIO_1();
}

void tm1638_init(void) {
	uint8_t i;
    statusTl = TM1638_STATUS_TL_NOT_INIT;
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
        default: break;
    }
    return statusTl;
}
