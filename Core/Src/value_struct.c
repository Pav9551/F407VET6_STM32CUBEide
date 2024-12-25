/*
 * value_struct.c
 *
 *  Created on: Dec 25, 2024
 *      Author: home
 */

#include "value_struct.h"
#include <stdio.h>
void printBinary(uint8_t byte) {
    for (int i = 7; i >= 0; --i) {
        printf("%d", (byte >> i) & 1);
    }
}
// Определение функции инициализации
void initValueStruct(value_Struct *vStruct) {
    if (vStruct == NULL) {
        return;
    }
    vStruct->ID = 0;
    vStruct->CUR_VALUE = 0;
    vStruct->NEW_VALUE = 0;
    vStruct->TRANSACTION = 0;
    vStruct->COUTN_TRANS = 0;
    vStruct->PRINT = 1;

}

// Определение функции печати значений структуры
void printValueStruct(const value_Struct *vStruct) {
    if (vStruct == NULL) {
        return;
    }

    if (vStruct->PRINT == 0) {
            return;
    }
    printf("\n\rCUR_VALUE: %u\n\r", vStruct->CUR_VALUE);
    printf("NEW_VALUE: %u\n\r", vStruct->NEW_VALUE);
    //printf("TRANSACTION: %u\n\r", vStruct->TRANSACTION);
    uint8_t lowByte = vStruct->TRANSACTION & 0xFF;
    printf("TRANSACTION: ");
    printBinary(lowByte);
    printf("\n\r");
    printf("COUTN_TRANS: %u\n\r", vStruct->COUTN_TRANS);

}


