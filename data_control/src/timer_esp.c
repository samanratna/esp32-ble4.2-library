//
// Created by peter on 12/24/2022.
//

#include "timer_esp.h"
#include "esp_timer.h"



void timer_esp_init() {
}

unsigned long long timer_esp_getTimeInMillis() {
    return esp_timer_get_time() / 1000;
    }
