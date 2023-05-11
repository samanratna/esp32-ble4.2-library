//
// Created by peter on 12/21/2022.
//

#include "esp_vcu.h"
#include <stdint.h>
#include <stddef.h>
#include <esp_vcu_communication.h>
#define ESP_VCU_PRIORITIES 3

TaskQue* esp_vcu_taskQue[ESP_VCU_PRIORITIES];
TaskQue* esp_vcu_errTaskQue[ESP_VCU_PRIORITIES];

/**
 * It initiates all the tasks required
 * (!!! should be called very first)
 */
void esp_vcu_init(){
    for(int i=0;i<ESP_VCU_PRIORITIES;i++)
        esp_vcu_taskQue[i] = task_que_create();
    for(int i=0;i<ESP_VCU_PRIORITIES;i++)
        esp_vcu_errTaskQue[i] = task_que_create();
}

/**
 * Add the task in task_que according to priority
 * @param priority      : priority
 * @param task          : task
 */
void esp_vcu_addTask(int priority,Task task){
    if(task_que_isExist(esp_vcu_taskQue[priority], task))
        return;
    if(task_que_isExist(esp_vcu_errTaskQue[priority], task))
        return;
    task_que_push(esp_vcu_taskQue[priority], task);
}

/**
 * It should be called in timer for executing all the task
 */
void esp_vcu_exec(){
    for(int i=0;i<ESP_VCU_PRIORITIES;i++){
        if(esp_vcu_taskQue[i]->size != 0){
            Task task = task_que_get(esp_vcu_taskQue[i]);
            if(task==NULL)
                return;
            ESP_VCU_Status status = task();
            if(status==ESP_VCU_BUSY)
                return;

            if(status==ESP_VCU_FAILED) {
                if(task_que_isExist(esp_vcu_errTaskQue[i], task))
                    break;
                task_que_push(esp_vcu_errTaskQue[i], task);
            }

            task_que_pop(esp_vcu_taskQue[i]);
            return;
        }
    }
}

/**
 * It try to fix error
 */
void esp_vcu_errExec(){
    for(int i=0;i<ESP_VCU_PRIORITIES;i++){
        if(esp_vcu_errTaskQue[i]->size != 0){
            Task task = task_que_get(esp_vcu_errTaskQue[i]);
            if(task==NULL)
                return;
            ESP_VCU_Status status = task();
            if(status==ESP_VCU_BUSY)
                return;

            if(status==ESP_VCU_OK)
                task_que_pop(esp_vcu_errTaskQue[i]);
        }
    }
}

/**
 * Return leaked memory of each task que with respect to priority
 * @param priority          : priority of task_que
 * @return                  : memory leaked of each task_que
 */
int esp_vcu_getLeakedMemory(int priority){
    return task_que_getLeakedBytes(esp_vcu_taskQue[priority]);
}

/**
 * Return leaked memory of each error task que with respect to priority
 * @param priority          : priority of err_task_que
 * @return                  : memory leaked of each err_task_que
 */
int esp_vcu_getErrLeakedMemory(int priority){
    return task_que_getLeakedBytes(esp_vcu_errTaskQue[priority]);
}

/**
 * Return total leaked memory
 * @return      : Leaked memory in bytes
 */
int esp_vcu_getTotalLeakedMemory(){
    int sum = 0;
    for(int i=0;i<ESP_VCU_PRIORITIES;i++)
        sum+= esp_vcu_getLeakedMemory(i)+ esp_vcu_getErrLeakedMemory(i);
    return sum;
}