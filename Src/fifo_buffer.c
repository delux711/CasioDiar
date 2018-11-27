#include "fifo_buffer.h"

struct FIFO_BUFF_CONFIG {
    uint8_t *buff;
    uint8_t sizeOfBuff;
    uint8_t pRecStart;
    uint8_t pRecEnd;
};

static uint8_t FIFO_pEnd = 0u;
static uint8_t FIFO_pStart = 0u;
static uint8_t FIFO_sizeOfBuff;
static uint8_t *FIFO_recBuff;
static uint8_t FIFO_actualConfig;
static struct FIFO_BUFF_CONFIG FIFO_configurations[FIFO_COUNT_OF_CONFIGURATIONS];

void FIFO_init(uint8_t configNum, uint8_t *buff, uint8_t sizeOfBuff) {
    FIFO_actualConfig = configNum;
    FIFO_configurations[configNum].buff = buff;
    FIFO_configurations[configNum].sizeOfBuff = sizeOfBuff;
    FIFO_configurations[configNum].pRecStart = 0u;
    FIFO_configurations[configNum].pRecEnd = 0u;
    FIFO_changeConfig(configNum);
    FIFO_pStart = 0u;
    FIFO_pEnd = 0u;
}

void FIFO_changeConfig(uint8_t configNum) {
    FIFO_configurations[FIFO_actualConfig].pRecStart = FIFO_pStart;
    FIFO_configurations[FIFO_actualConfig].pRecEnd = FIFO_pEnd;
    FIFO_recBuff = FIFO_configurations[configNum].buff;
    FIFO_sizeOfBuff = FIFO_configurations[configNum].sizeOfBuff;
    FIFO_pStart = FIFO_configurations[configNum].pRecStart;
    FIFO_pEnd = FIFO_configurations[configNum].pRecEnd;
    FIFO_actualConfig = configNum;
}

uint8_t FIFO_getData(void) {
    uint8_t ret;
    ret = FIFO_pStart;
    if(FIFO_pStart != FIFO_pEnd) {
        FIFO_pStart++;
        if(FIFO_sizeOfBuff <= FIFO_pStart) {
            FIFO_pStart = 0;
            if(FIFO_sizeOfBuff <= FIFO_pEnd) {
                FIFO_pEnd = 0;
            }
        }
    } else {
        if(0u == FIFO_pStart) {
            ret = FIFO_sizeOfBuff - 1U;
        } else {
            ret = FIFO_pStart - 1u;
        }
    }
    ret = FIFO_recBuff[ret];
    return ret;
}
    
bool FIFO_putData(uint8_t ch) {
    if(true == FIFO_isPutDataReady()) {
        if(FIFO_sizeOfBuff <= FIFO_pEnd) {
            FIFO_pEnd = 0u;
        }
        FIFO_recBuff[FIFO_pEnd] = ch;
        FIFO_pEnd++;
        return true;
    }
    return false;
}

bool FIFO_isUnreadData(void) {
    return FIFO_pStart != FIFO_pEnd;
}

bool FIFO_isPutDataReady(void) {
    return FIFO_pStart != (FIFO_pEnd + 1u);
}
