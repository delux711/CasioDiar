#ifndef _FIFO_BUFFER
#define _FIFO_BUFFER

#include <stdint.h>
#include <stdbool.h>

/**
    Funkcia FIFO_getData() vzdy vracia najstarsiu ulozenu hodnotu. Ak uz nove hodnoty nie su, tak sa vrati posledna precitana.
    Ako pouzit FIFO buffer pri pouziti dvoch buffrov a prepinanie medzi nimi:
    1. Niekde vytvorit 2 buffery a inkludovat kniznicu
            #include "fifo_buffer.h"
            static uint8_t SPu1_recBuff[BUFF_MAX];      // BUFF_MAX napr. 10    (buff 0)
            static uint8_t SPu1_recBuff2[BUFF_MAX2];    // BUFF_MAX2 napr. 5    (buff 1)
    2. Inicializacia
            FIFO_init(0u, SPu1_recBuff, BUFF_MAX);      // init buff 0
            FIFO_init(1u, SPu1_recBuff2, BUFF_MAX2);    // init buff 1 - buff1 je teraz aktivy
    3. Ak chcem pracovat s buff 0 tak sa musim prepnut
            FIFO_changeConfig(0u);
    4. Ulozit nejake data
            FIFO_putData(dataFirst);
            FIFO_putData(data2);
            ...
            FIFO_putData(dataX);
            FIFO_changeConfig(1u);  // zmena prace na buff1
            FIFO_putData(dataForBuff1);
    5.  Vycitat data
            readData = FIFO_getData();  // v readData je teraz dataForBuff1
            readData = FIFO_getData();  // v readData je opat dataForBuff1 pretoze bola zapisana len jedna hodnota
    6.  Ak chcem citat z buff 0 tak sa musim prepnut
            FIFO_changeConfig(0u);
            readDataFomBuff0 = FIFO_getData();  // v readDataFomBuff0 je teraz dataFirst
            readDataFomBuff0 = FIFO_getData();  // v readDataFomBuff0 je teraz data2
        
*/

/**
    Kolko mam dokopy buffrov. Minimalne 1.
*/

#define FIFO_COUNT_OF_CONFIGURATIONS    (2u)

/**
    Funkcia po ukonceni prepne FIFO na aktualne nastaveny buffer. Predchadzajuca konfiguracia na danej configNum
    konfiguracii bude prepisana a stratena.
    
    @param configNum    Buffer ktory bude inicializovany/reinicializovany
    @param buff         Smernik na buffer ktory bude mapovany pre danu configNum konfiguraciu
    @param sizeOfBuff   Maximalna velkost namapovaneho buffra
*/
extern void FIFO_init(uint8_t configNum, uint8_t *buff, uint8_t sizeOfBuff);
/**
    @param configNum    Buffer s ktorym sa bude pracovat.
*/
extern void FIFO_changeConfig(uint8_t configNum);
/**
    @note   Funkcia pri preplnenom buffry neulozi data ch! Vstupne data bude stratene!
    @return true - ak sa data ulozili do buffra
            false - ak nebolo mozne data ulozit kvoli preplnenosti buffra
*/
extern bool FIFO_putData(uint8_t ch);
/**
    @return true - ak existuju neprecitane data v buffry
*/
extern bool FIFO_isUnreadData(void);
/**
    @return true - ak budu po volani funkcie FIFO_putData ulozene data do buffra
            false - ak su v buffry neprecitane data a nie je kde zapisat dalsie
*/
extern bool FIFO_isPutDataReady(void);
extern uint8_t FIFO_getData(void);

#endif // _FIFO_BUFFER
