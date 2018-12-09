#include "tm1638.h"

void tm1638_sendCommand(uint8_t comm);
void tm1638_sendData(uint8_t data);
void tm1638_setAddress(uint8_t address, bool stop);
uint8_t tm_convToDigit(uint8_t ch);

#define TM_STB_0()	do { GPIOE->BSRR |= GPIO_BSRR_BR13; } while(0u)
#define TM_STB_1()	do { GPIOE->BSRR |= GPIO_BSRR_BS13; } while(0u)

#define TM_CLK_0()	do { GPIOE->BSRR |= GPIO_BSRR_BR14; } while(0u)
#define TM_CLK_1()	do { GPIOE->BSRR |= GPIO_BSRR_BS14; } while(0u)

#define TM_DIO_IN()		do { GPIOE->MODER &= ~GPIO_MODER_MODE15_Msk; } while(0u)
#define TM_DIO_OUT()	do { GPIOE->MODER |= (1u << GPIO_MODER_MODE15_Pos); } while(0u)
#define TM_DIO_0()		do { GPIOE->BSRR |= GPIO_BSRR_BR15; } while(0u)
#define TM_DIO_1()		do { GPIOE->BSRR |= GPIO_BSRR_BS15; } while(0u)

uint8_t tm_convToDigit(uint8_t ch) {
	uint8_t ret;
	switch(ch) {
		case '0': ret = 0x3Fu; break;
		case '1': ret = 0x06u; break;
		case '2': ret = 0x5Bu; break;
		case '3': ret = 0x4Fu; break;
		case '4': ret = 0x66u; break;
		case '5': ret = 0x6Du; break;
		case '6': ret = 0x7Du; break;
		case '7': ret = 0x07u; break;
		case '8': ret = 0x7Fu; break;
		case '9': ret = 0x6Fu; break;
	//	case : ret = 0xu; break;
		default: ret = 0u; break;
	}
	return ret;
}

void tm1638_show(uint8_t *buff) {
	uint8_t i, j;
	uint8_t temp[16];
	j = 0u;
	for(i = 0u; i < 8u; i++) {
		temp[j++] = tm_convToDigit(buff[i]);
		temp[j++] = 0x01;	// led on
	}
	tm1638_setAddress(0u, false);
	for(i = 0u; i < 16u; i++) {
		tm1638_sendData(temp[i]);
	}
	TM_STB_1();
}

void tm1638_showPos(uint8_t position, uint8_t ch) {
	tm1638_setAddress(position + 1u, false);
	tm1638_sendData(tm_convToDigit(ch));
	TM_STB_1();
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

void tm1638_init(void) {
	uint8_t i;
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
	
	tm1638_sendCommand(TM1638_COMMAND_WRITE_DATA);
	tm1638_sendCommand(TM1638_COMMAND_PULSE_10_16);
	tm1638_sendCommand(TM1638_COMMAND_LCD_ON);
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
