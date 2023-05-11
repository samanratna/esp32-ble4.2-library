//
// Created by peter on 12/13/2022.
//

#include "que.h"
#include "stdlib.h"

int que_leak_track = 0;
static int que_id_count = 0;

/**
 * Create que and return pointer to it
 * @return          : Pointer to que (!!!dynamically allocated : que_delete(que) must be called)
 *                  : NULL if heap is full
 */
Que *que_create() {
    Que *que = (Que *) malloc(sizeof(Que));
    if(que==NULL)
        return NULL;
    que_leak_track += sizeof(Que);

    que->front_content = NULL;
    que->rear_content = NULL;
    que->size = 0;
    que->leaked_bytes = 0;
    return que;
}

/**
 * Push the value in que
 * @param que       : Pointer to que where content is to be added
 * @param value      : QueType to be pushed
 * @return          : Pointer to same que
 *                  : NULL if heap is full
 */
Que *que_push(Que *que, QueType value) {
    if(que_isExist(que,value))
        return que;
    QueContent *content = (QueContent *) malloc(sizeof(QueContent));
    if(content==NULL)
        return NULL;
    int size = sizeof(QueContent);
    que_leak_track += size;
    que->leaked_bytes += size;

    content->id = que_id_count++;
    content->value = value;
    content->next_content = NULL;

    if (que->size == 0)
        que->front_content = content;
    else
        que->rear_content->next_content = content;

    que->rear_content = content;
    que->size++;

    return que;
}

/**
 * Return the first value in que
 * @param que       : Que
 * @return          : First value in que
 *                  : NULL is que is empty
 */
QueType que_get(Que* que){
    if((que->size)<=0)
        return NULL;
    return que->front_content->value;
}

/**
 * Remove the first value of que
 * @param que       : Pointer to the que
 * @return          : Pointer to same que
 */
Que *que_pop(Que *que) {
    if (que->size <= 0)
        return que;

    QueContent *temp = que->front_content;
    que->front_content = temp->next_content;

    free((QueContent *) temp);
    int size = sizeof(QueContent);
    que_leak_track -= size;
    que->leaked_bytes -= size;

    que->size--;
    if (que->size == 0) {
        que->front_content = NULL;
        que->rear_content = NULL;
    }
    return que;
}

/**
 * Delete the whole que
 * @param que       : Pointer to que
 */
void que_delete(Que *que) {
    if (que == NULL)
        return;
    int len = que->size;
    for (int i = 0; i < len; i++)
        que_pop(que);
    free(que);
    que_leak_track -= sizeof(Que);
}

/**
 * Print the que
 * @param que       : Que
 * @param print     : callback combiner_test_print function (i.e printf)
 */
void que_print(Que* que, int (*print)(const char *format, ...)) {
    QueContent *content = que->front_content;
    print("[ ");
    for (int i = 0; i < que->size; ++i) {
        print("%d ", content->id);
        content = content->next_content;
    }
    print("]\n");
}

/**
 * This check if value exist or not
 * @param que   : Que
 * @param value  : QueType ot be compared
 * @return      : 1 for exist
 *              : 0 for doesn't exist
 */
int que_isExist(Que* que, QueType value){
    int len = que->size;
    QueContent *temp = que->front_content;
    for(int i = 0; i < len; i++){
        if(temp->value==value)
            return 1;
        temp = temp->next_content;
    }
    return 0;
}

/**
 * Return the leaked bytes
 * @param que       : QueType que or NULL
 * return 			: Leaked bytes
 */
int que_getLeakedBytes(Que* que){
    if(que==NULL)
        return que_leak_track;
    return que->leaked_bytes;
}
