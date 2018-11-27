//#include "Board_LED.h"                  // ::Board Support:LED
//#include "Board_Buttons.h"              // ::Board Support:Buttons
#include "getJoystick.h"
#include "lcd.h"
#include "myRTC.h"
#include "serialPort.h"
#include "casioDiar.h"
#include "timerLib.h"
#include "serialPort_u1.h"
#include <stdio.h>
#include <stdbool.h>
#include "../hi2c/hi2c.h"

// SYSCLK - 4MHz
// MSI - 4MHz
// HSI16 - 16MHz
// 100: HSE clock selected - OFF
// 101: Main PLL clock selected - OFF
// 110: LSI clock selected - 32kHz
// 111: LSE clock selected - OFF

void run_in_ram(void);
void load_ramcode(void);
extern char Image$$RW_CODE$$Base;		// kvoli testovaniu volania funkcie z RAM
extern char Image$$RW_CODE$$Length; // kvoli testovaniu volania funkcie z RAM
extern char Load$$RW_CODE$$Base;		// kvoli testovaniu volania funkcie z RAM
__attribute__((section("TEMPDATASECTION"), zero_init)) uint8_t foo;


typedef union _BMP180_calVal {
    uint16_t buff[11];
    struct _calBytes {
        int16_t AC1;
        int16_t AC2;
        int16_t AC3;
        uint16_t AC4;
        uint16_t AC5;
        uint16_t AC6;
        int16_t B1;
        int16_t B2;
        int16_t MB;
        int16_t MC;
        int16_t MD;
    } calBytes;
} BMP180_calVal;

typedef union _BMP180_temp {
    uint8_t buff[2];
    uint16_t temp;
    struct _bytes {
        uint8_t MSB;
        uint8_t LSB;
    } bytes;
} BMP180_temp;
/*
typedef union _BMP180_pres {
    uint8_t buff[3];
    int32_t pressure;
    struct _bytes {
        uint8_t MSB;
        uint8_t LSB;
        uint8_t XLSB;
    } bytes;
} BMP180_pres;
*/
typedef struct _BMP180 {
    BMP180_calVal calVal;
    long UT;
    long UP;
    long X1;
    long X2;
    long X3;
    long B5;
    long B6;
    long T;
} BMP180;


BMP180 sensorPresure;
uint8_t HI2C_readByte(uint8_t addr, bool stop);
bool HI2C_writeByte(uint8_t addr, bool stop, uint8_t data);
bool HI2C_writeAddr(uint8_t addr, bool stop);
void BMP180_readCalData(void);
void BMP180_readTemp(void);
void BMP180_reset(void);
bool BMP180_doneSample(void);

bool BMP180_isDoneSample(void) {
    bool ret;
    ret = true;
    if(0u != (0x20u & HI2C_readByte(0xF4u, true))) { // Sco (register F4h <5>): Start of conversion. The value of this bit stays “1” during conversion 
        ret = false;                                   // and is reset to “0” after conversion is complete (data registers are filled). 
    }
    return ret;
}

bool HI2C_writeAddr(uint8_t addr, bool stop) {
    bool ret;
    ret = false;
    if(true == HI2C0_bSetAddr(0xEFu & 0xFE)) { // write
        if(true == HI2C0_bSetTxData(addr, stop)) { // write address
            ret = true;
        }
    }
    return ret;
}

bool HI2C_writeByte(uint8_t addr, bool stop, uint8_t data) {
    bool ret;
    ret = false;
    if(true == HI2C_writeAddr(addr, false)) { // write
        if(true == HI2C0_bSetTxData(data, stop)) { // write address
            ret = true;
        }
    }
    return ret;
}

void BMP180_reset(void) {
    (void)HI2C_writeByte(0xE0u, true, 0xB6u);   // 0xB6-reset sequence
}
        
        
uint8_t HI2C_readByte(uint8_t addr, bool stop) {
    uint8_t ret;
    ret = 0u;
    if(true == HI2C0_bSetAddr(0xEFu & 0xFE)) { // write
        if(true == HI2C0_bSetTxData(addr, true)) { // write address
            if(true == HI2C0_bSetAddr(0xEFu)) { // read
                ret = HI2C0_vTriggerReceive(stop);
            }
        }
    }
    return ret;
}
void BMP180_readCalData(void) {
    uint8_t i;
    if(true == HI2C0_bSetAddr(0xEFu & 0xFE)) { // write
        if(true == HI2C0_bSetTxData(0xAAu, true)) { // write address with calibration data
            if(true == HI2C0_bSetAddr(0xEFu)) { // read
                for(i = 0; i < 10u; i++) {
                    sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
                    sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(false);
                }
                sensorPresure.calVal.buff[i] = (HI2C0_vTriggerReceive(false) << 8u);
                sensorPresure.calVal.buff[i] |= HI2C0_vTriggerReceive(true);
            }
        }
    }
}
void BMP180_readTemp(void) {
    //uint8_t i;
    if(true == HI2C_writeByte(0xF4u, true, 0x2Eu)) {
    //if(true == HI2C0_bSetAddr(0xEFu & 0xFE)) { // write
      //  if(true == HI2C0_bSetTxData(0xF4u, false)) { // write address with start of temp measurement
        //    if(true == HI2C0_bSetTxData(0x2Eu, true)) { // start measurement temp
                // delay min 4,5ms
                //for(i = 0u; i < 255u; i++);
                while(false == BMP180_isDoneSample());
                sensorPresure.UT = (long)(HI2C_readByte(0xF6u, false) << 8u); // address with temparature
                sensorPresure.UT |= HI2C0_vTriggerReceive(true);
                sensorPresure.X1 = sensorPresure.UT - sensorPresure.calVal.calBytes.AC6;
                sensorPresure.X1 *= sensorPresure.calVal.calBytes.AC5;
                sensorPresure.X1 /= 32768;  // 2^15
                sensorPresure.X2 = sensorPresure.calVal.calBytes.MC * 2048; // 2^11
                sensorPresure.X2 /= sensorPresure.X1 + sensorPresure.calVal.calBytes.MD;
                sensorPresure.B5 = sensorPresure.X1 + sensorPresure.X2;
                sensorPresure.T = (sensorPresure.B5 + 8u) / 16u;
    /*        }
        }*/
    }
}

/*
static uint64_t *lcdRam1;
static uint64_t *lcdRam2;
static uint64_t *lcdRam3;
static uint64_t *lcdRam4;
static uint8_t data[] = { 'A', 255, 255, 255, 255, 255 };*/
static uint8_t testDiar[] = { "Ing.Pavol Pusztai, Slovinska 1, 05342 Krompachy." };
static uint8_t receiveBuff[50];
static uint8_t receiveP = 0;
int main(void) {
    bool bBmp180present;
    bool lcdIsShift;
	bool cdNewData = false;
    uint8_t i, ch;
	uint8_t buff[50];
    uint8_t cd_buff[300];
    uint32_t count = 0;
    uint8_t stredTmp = 0;
    
    bBmp180present = false;
    
	SPu1_init();
	
    load_ramcode();
    tl_Init();
    TIM_delayInit();
    MX_LCD_Init();
    LCD_GLASS_Clear();
    myRtcInit();
    RCC->CFGR |= (4u << RCC_CFGR_MCOPRE_Pos); // MCO / 16 IF 4
    SP_init();
    
    
    HI2C0_vInit(0u);
    for(count = 0u; count < 10000u; count++) {
        BMP180_reset();
        i = HI2C_readByte(0xD0u, true);   // 0xD0u - Chip-id
        if(0x55u == i) {
            bBmp180present = true;
            BMP180_readCalData();
            BMP180_readTemp();
            break;
        }
    }
    count = 0u;

    //LCD_GLASS_DisplayString((uint8_t *)"A");
    /*	LCD_GLASS_DisplayString(data);
    lcdRam1 = (uint64_t*) &LCD->RAM[0];
    lcdRam2 = (uint64_t*) &LCD->RAM[2];
    lcdRam3 = (uint64_t*) &LCD->RAM[4];
    lcdRam4 = (uint64_t*) &LCD->RAM[6];
    if(*lcdRam1 && *lcdRam2 && *lcdRam3 && *lcdRam4) {
    }*/
    printf("\nTestovanie\n");
    sprintf((char*)buff, "%06d", count);
    LCD_GLASS_DisplayString(buff);
    if(tl_getTl().stred) {
		i = 0u;
		for(ch = ('a'-1U); ch < ('z'+1u); ch++) {
			buff[i++] = ch;
		}
		buff[i] = '\0';
		LCD_GLASS_DisplayString(buff);
    }
    while(1) {
        TIM_handleTask();
        lcdIsShift = MX_LCD_Task();
        if(SP_isNewData() == true) {
            ch = SP_getData();
            if((ch == '\r') || (18u < receiveP)) {
                receiveBuff[receiveP++] = '\0';
                //receiveBuff[7] = '\0';
							LCD_GLASS_DisplayString(receiveBuff);
                (void)SP_sendString(receiveBuff);
                receiveP = 0u;
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 2000u);
            } else {
                receiveBuff[receiveP++] = ch;
            }
        }
        if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW) == true) {
            if((myRtcIsNewTime() == true) && (lcdIsShift == false)) {
                myRtcSaveActualTime();
                myRtcGetTimeString(buff);
                LCD_GLASS_DisplayString(buff);
                //LCD_GLASS_DisplayStringTime(buff);
            }
        }
        switch(CD_task()) {
            case CD_STATE_RECEIVING:    run_in_ram();   break;
            case CD_STATE_SENDING:      led2_on();      break;
            case CD_STATE_SLEEP:
                led1_off();
                led2_off();
                break;
            case CD_STATE_SENDED_CALENDAR:  LCD_GLASS_DisplayString((uint8_t*) "CALENDAR ");  break;
            case CD_STATE_SENDED_TELEPHONE: LCD_GLASS_DisplayString((uint8_t*) "TELEPHONE "); break;
            case CD_STATE_SENDED_NOTE:      LCD_GLASS_DisplayString((uint8_t*) "NOTE ");      break;
            case CD_STATE_SENDED_SCHEDULE:  LCD_GLASS_DisplayString((uint8_t*) "SCHEDULE ");  break;
            case CD_STATE_SENDED_REMINDER:  LCD_GLASS_DisplayString((uint8_t*) "REMINDER ");  break;
            case CD_STATE_SENDED_REMINDER2: LCD_GLASS_DisplayString((uint8_t*) "REMINDER ");  break;
            case CD_STATE_SENDED_FREE_FILE: LCD_GLASS_DisplayString((uint8_t*) "FREE FILE "); break;
            case CD_STATE_SENDED_DATA:
                /*
                myRtcSetTime(CD_getBuffer());
                sprintf((char*)buff, "Set time to: %s", CD_getBuffer());
                LCD_GLASS_DisplayString(buff);*/
                cdNewData = true;
                TIM_delaySetTimer(DELAY_MAIN_CASIO_NEW_DATA, 1500u);
                break;
            case CD_STATE_NOT_INIT: CD_setUserBuffer(cd_buff, sizeof(cd_buff)); break;
            default: break;
        }
        if((true == cdNewData) && (true == TIM_delayIsTimerDown(DELAY_MAIN_LCD_SHOW))) {
            cdNewData = false;
            sprintf((char*)buff, "Set time to: %s", cd_buff);
            LCD_GLASS_DisplayString(buff);
        }

		if(tl_getTlSample().hore) {
			//led1_on();
			foo++;
			CD_sendToDiarConst(0u);
            //run_in_ram();
            
		} else {
			//led1_off();
		}
        if(tl_getTlSample().lavo) {
            myRtcGetTime((uint8_t *)buff);
            sprintf((char*)&buff[6], "-%06d", ++count);
            CD_sendToDiarConst((uint8_t *)buff);
		}
        if(tl_getTlSample().pravo) {
            //CD_sendToDiarConst(testDiar);
            (void)CD_receive();
		}
        if(tl_getTl().dole) {
			CD_senToDiarEndCommunication();
            LCD_GLASS_DisplayString(testDiar);
		} else {
		}
		if(tl_getTl().stred) {
			stredTmp = 1;
			sprintf((char*)buff, "%06d", ++count);
			LCD_GLASS_DisplayString((uint8_t*) buff);
		} else if (stredTmp) {
			if(USART2->ISR & USART_ISR_TXE) {
				stredTmp = 0;
				printf("Count: %d", count);
			}
		}
        
        if(true == bBmp180present) {
            if(TIM_delayIsTimerDown(DELAY_MAIN_LCD_TEMP_SHOW) == true) {
                TIM_delaySetTimer(DELAY_MAIN_LCD_TEMP_SHOW, 4000u);
                TIM_delaySetTimer(DELAY_MAIN_LCD_SHOW, 1000u);
                BMP180_readTemp();
                //sprintf((char*)buff, "%06Ld°C", sensorPresure.T);
                sprintf((char*)buff, "%d.%d°C", (int8_t)(sensorPresure.T / 10u), (sensorPresure.T / 100u));
                LCD_GLASS_DisplayString((uint8_t*) buff);
            }
        }
	} // while(1);
}

// kvoli testovaniu volania funkcie z RAM
__attribute__((section("RAMCODESECTION"))) void run_in_ram(void) {
  __nop();
	foo++;
	led1_on();
}

// kvoli testovaniu volania funkcie z RAM
void load_ramcode(void) {
	uint32_t i;
	char *from, *to;
	to = &Image$$RW_CODE$$Base;
	from = &Load$$RW_CODE$$Base;
	for(i = 0; i < (size_t)&Image$$RW_CODE$$Length; i++) {
		to[i] = from[i];
	}
}
