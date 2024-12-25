/*
 * value_struct.h
 *
 *  Created on: Dec 25, 2024
 *      Author: home
 */

#ifndef INC_VALUE_STRUCT_H_
#define INC_VALUE_STRUCT_H_

#include <stdint.h>

// Определение структуры
typedef struct {
	volatile uint32_t ID;
    volatile uint32_t CUR_VALUE;
    volatile uint32_t NEW_VALUE;
    volatile uint32_t TRANSACTION;
    volatile uint32_t COUTN_TRANS;
    volatile uint32_t PRINT;
} value_Struct;

// Объявление функции инициализации
void initValueStruct(value_Struct *vStruct);

// Объявление функции печати значений структуры
void printValueStruct(const value_Struct *vStruct);

#endif /* INC_VALUE_STRUCT_H_ */
