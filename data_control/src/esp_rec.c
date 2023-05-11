//
// Created by peter on 12/25/2022.
//

#include "esp_rec.h"
#include "stdio.h"
#include "stdlib.h"

#include "map.h"
#include "que.h"
#include "combiner.h"
#include "timer_esp.h"



#define ESP_REC_PRIORITIES      1
#define ESP_REC_WAIT_TIME       4  //In sec
#define ESP_REC_QUE_WAIT_TIME   4

Map *esp_rec_map;//incomplete data
Que *esp_rec_que[ESP_REC_PRIORITIES];//completed data
int esp_memory_leaked;
bool esp_init_succeed;

static float esp_que_time_elapsed;

int (*esp_rec_ack)(uint32_t,const uint8_t*,uint8_t);

void esp_rec_print(EspRecData data) {
//    printf(" %d :[ ", data.can_id);
//    for (int i = 0; i < data.count; i++)
//        printf("%d ", data.bytes[i]);
//    printf("]\n");
}

/**
 * This should be override to help in debugging
 * @param title         : title of error
 * @param reason        : reason of error
 */
void esp_rec_debug(const char *title, const char *reason) {
    printf("(ESP_REC)%s :: %s\n", title, reason);
}

/**
 * This should be called once at first to initiate everything related to receiving
 * @param canSend       : pointer to the can send function
 * @return              : true if succeed
 *                      : false if failed
 */
int esp_rec_init(int (*canSend)(uint32_t, const uint8_t*, uint8_t)) {
    esp_rec_ack = canSend;
    esp_init_succeed = false;
    esp_rec_map = map_create();
    if (esp_rec_map == NULL) {
        esp_rec_debug("esp_rec_map", "NULL(Heap is full)");
        return false;
    }
    for(int i = 0; i < ESP_REC_PRIORITIES; i++) {
        esp_rec_que[i] = que_create();
        if (esp_rec_que[i] == NULL) {
            esp_rec_debug("esp_rec_que", "NULL(Heap is full)");
            return 0;
        }
    }
    esp_memory_leaked = 0;
    esp_init_succeed = true;
    timer_esp_init();
    return 1;
}

/**
 *  * This should be called in timer
 * @return              : can ID of data from que
 *                      : 0 for empty que
 */
uint32_t esp_rec_getCanID() {
    EspRecData *data;
    for(int i = 0; i < ESP_REC_PRIORITIES; i++){
        data = que_get(esp_rec_que[i]);
        if(data == NULL)
            continue;
        return data -> can_id;
    }
    return 0;
}


/**
 * This should be called in timer
 * @param struct_data   : get struct data
 * @return              : true for success
 *                      : false for no data
 */
bool esp_rec_getData(void *struct_data) {
    EspRecData *data;
    for(int i=0;i<ESP_REC_PRIORITIES;i++){
        data = que_get(esp_rec_que[i]);
        if(data==NULL)
            continue;
        if(struct_data!=NULL)
            combiner_bytesToGenericData(data->bytes, struct_data, data->len);
        que_pop(esp_rec_que[i]);
        free(data->bytes);
        esp_memory_leaked-=data->len;
        free(data);
        esp_memory_leaked-= sizeof(EspRecData);

        esp_que_time_elapsed = (float) timer_esp_getTimeInMillis() / 1000.0f;
        return true;
    }
    return false;
}



/**
 * Set the can send function which may be call back
 * @param canSend       : pointer to the can send function
 */
void esp_rec_setCanSend(int (*canSend)(uint32_t,const uint8_t*,uint8_t)) {
    esp_rec_ack = canSend;
}

/**
 * This should be called everytime can receive interrupt
 * @param can_id        : Can id
 * @param data          : data
 * @param len           : size of data in bytes
 */
void esp_rec_interrupt(uint32_t can_id, const uint8_t *data, uint8_t len) {
    if (!esp_init_succeed)
        return;
    if (can_id == ESP_REC_START_ACK_ID) {
        //Communication start

        /**
         * Get data length and can id of data
         */
        uint8_t data_len = data[0];
        uint32_t data_can_id = data[4] | (data[3] << 8) | (data[2] << 16) | (data[1] << 24);

        /**
         * Allocate memory for incomplete data
         */
        EspRecData *incompleteData=NULL;
        if(map_get(esp_rec_map,data_can_id)!=NULL)
            return;
        incompleteData = malloc(sizeof(EspRecData));
        esp_memory_leaked += sizeof(EspRecData);
        incompleteData->can_id = data_can_id;
        incompleteData->count = 0;
        incompleteData->len = data_len;
        incompleteData->bytes = malloc(data_len);
        incompleteData->sec = ((float) timer_esp_getTimeInMillis()) / 1000.0f;
        esp_memory_leaked += data_len;

        /**
         * Put data in map
         */
        map_put(esp_rec_map, data_can_id, incompleteData);
        esp_rec_ack(ESP_REC_START_ACK_ID, data, len);
    } else if (can_id == ESP_REC_END_ACK_ID) {
        //End of communication

        /**
        * Get data length and can id of data
        */
        uint8_t data_len = data[0];
        uint32_t data_can_id = data[4] | (data[3] << 8) | (data[2] << 16) | (data[1] << 24);
        uint8_t priority = data[5];

        /**
         * Get completed data from map and push to que
         */
        EspRecData *completeData = map_get(esp_rec_map, data_can_id);
        if(completeData==NULL)
            return;
        completeData->count = data_len;

        completeData->sec = 0;
        que_push(esp_rec_que[priority], completeData);

        /**
         * Remove completed data from map
         */
        map_remove(esp_rec_map, data_can_id);

        esp_rec_ack(ESP_REC_END_ACK_ID, data, len);

//        esp_rec_print(*completeData);
    } else {
        //Data
        /**
         * Add data to incomplete data
         */
        EspRecData *incompleteData = map_get(esp_rec_map, can_id);
        if (incompleteData == NULL)
            return;
        for (int i = 0; i < len; ++i, incompleteData->count++)
            incompleteData->bytes[incompleteData->count] = data[i];
        incompleteData->sec = ((float) timer_esp_getTimeInMillis()) / 1000.0f;
        uint8_t d[8] = {incompleteData->count};
        esp_rec_ack(ESP_REC_DATA_ACK_ID, d, 1);

        if(incompleteData->count>incompleteData->len)
            printf("Incomplete Data Issue!\n");
    }
}

/**
 * This returns leaked memory
 * @return              : memory leaked in this file
 */
int esp_rec_getLeakedBytes() {
    return esp_memory_leaked;
}

/**
 * This function should be called in error timer fixing
 * It waits for data in map to finish communication for finite time and
 * then remove data from map to reduce overload
 */
void esp_rec_errorFixTimer() {
    int size = esp_rec_map->size;
    uint32_t keys[size];
    map_getKeys(esp_rec_map, keys);
//    for(int i=0;i<size;i++)
//        printf("%d ",keys[i]);
//    printf("\n");
    for (int i = 0; i < size; i++) {
        EspRecData *data = map_get(esp_rec_map, keys[i]);
        if (data == NULL)
            continue;
        float time_elapsed = data->sec;
        time_elapsed = (float) timer_esp_getTimeInMillis() / 1000.0f - time_elapsed;
        if (time_elapsed >= ESP_REC_WAIT_TIME) {
            map_remove(esp_rec_map, keys[i]);
            free(data->bytes);
            esp_memory_leaked-=data->len;
            free(data);
            esp_memory_leaked-= sizeof(EspRecData);
        }
    }

    size = 0;
    for(int i=0;i<ESP_REC_PRIORITIES;i++)
        size+=esp_rec_que[i]->size;

    if(size<=0){
        esp_que_time_elapsed = (float )timer_esp_getTimeInMillis()/1000.0f;
        return;
    }
    float time_elapsed = (float )timer_esp_getTimeInMillis()/1000.0f-esp_que_time_elapsed;
    if(time_elapsed>=ESP_REC_QUE_WAIT_TIME){
        esp_rec_getData(NULL);
    }
}