#ifndef _OBD_H_
#define _OBD_H_

#include "main.h"

extern bool OBD_ACTIVATE;

int ESP_FLASH_OBD();
void obd_task(void *arg);
void obd_can(uint32_t struct_id, uint32_t can_id, uint8_t data[], uint8_t data_len);

#endif