//
// Created by peter on 12/17/2022.
//
#include "main.h"
#include "combiner.h"

/**
 * This convert struct into bytes
 * @param data      : pointer to data to be converted
 * @param bytes     : bytes to store
 * @param len       : Size in bytes of struct
 */
void combiner_genericDataToBytes(void *data, uint8_t *bytes, int len) {
    memcpy(bytes, data, len);
}

/**
 * This converts bytes to struct
 * @param data      : pointer to data
 * @param bytes     : Bytes to be converted
 * @param len       : Size of bytes or struct to be return
 */
void combiner_bytesToGenericData(uint8_t *bytes, void *data, int len) {
    memcpy(data, bytes, len);
}
