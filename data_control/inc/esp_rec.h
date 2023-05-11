//
// Created by peter on 12/25/2022.
//

#ifndef COMMUNICATION_ESP_REC_H
#define COMMUNICATION_ESP_REC_H

#include <stdint.h>
#include "stdbool.h"

typedef struct
{
    uint32_t can_id;
    uint8_t *bytes;
    uint8_t count;
    uint8_t len;
    float sec;
} EspRecData;

void esp_rec_debug(const char *, const char *);

int esp_rec_init(int (*)(uint32_t, const uint8_t *, uint8_t));

uint32_t esp_rec_getCanID();

bool esp_rec_getData(void *);

void esp_rec_setAck(int (*ack)(uint32_t, const uint8_t *, uint8_t));

void esp_rec_interrupt(uint32_t, const uint8_t *, uint8_t);

void esp_rec_test();

int esp_rec_getLeakedBytes();

void esp_rec_errorFixTimer();

#endif // COMMUNICATION_ESP_REC_H
