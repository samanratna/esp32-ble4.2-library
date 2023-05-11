#ifndef INC_MAIN_H_
#define INC_MAIN_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "string.h"
#include "driver/twai.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"

#include "vcu.h"
#include "esp_vcu.h"
#include "vcu_esp.h"
#include "esp_rec.h"
#include "esp_vcu_communication.h"
#include "ble.h"
#include "OTA.h"
#include "OBD.h"
#include "can.h"
#include "sim7600.h"
#include "credentials.h"

#endif