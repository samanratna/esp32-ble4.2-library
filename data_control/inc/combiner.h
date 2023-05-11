//
// Created by peter on 12/17/2022.
//

#ifndef PETER_PRACTICE_C_COMBINER_H
#define PETER_PRACTICE_C_COMBINER_H


#include <stdint.h>
#include "main.h"

void combiner_genericDataToBytes(void *, uint8_t *, int);
void combiner_bytesToGenericData(uint8_t *, void *, int);
typedef union { 
    void *data;
    uint8_t *bytes;
} combiner_Combiner;

uint8_t *combiner_structToBytes(void *, int);
void *combiner_bytesToStruct(uint8_t *, int);
void combiner_test();

#endif //PETER_PRACTICE_C_COMBINER_H
