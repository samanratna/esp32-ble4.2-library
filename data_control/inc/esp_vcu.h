//
// Created by peter on 12/21/2022.
//

#ifndef COMMUNICATION_ESP_VCU_H
#define COMMUNICATION_ESP_VCU_H
#include "task_que.h"

// typedef enum{
//     ESP_VCU_OK = 1,
//     ESP_VCU_FAILED = 0,
//     ESP_VCU_BUSY = -1
// }ESP_VCU_Status;

void esp_vcu_init();
void esp_vcu_addTask(int,Task);
void esp_vcu_exec();

void esp_vcu_test();
void esp_vcu_errExec();

int esp_vcu_getLeakedMemory(int);
int esp_vcu_getErrLeakedMemory(int);
int esp_vcu_getTotalLeakedMemory();

#endif //COMMUNICATION_ESP_VCU_H
