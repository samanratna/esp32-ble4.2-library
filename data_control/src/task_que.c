//
// Created by peter on 12/13/2022.
//

#include "task_que.h"
#include "stdlib.h"

int task_que_leak_track = 0;
static int task_que_id_count = 0;

/**
 * Create task_que and return pointer to it
 * @return          : Pointer to task_que (!!!dynamically allocated : task_que_delete(task_que) must be called)
 *                  : NULL if heap is full
 */
TaskQue *task_que_create() {
    TaskQue *que = (TaskQue *) malloc(sizeof(TaskQue));
    if(que==NULL)
        return NULL;
    task_que_leak_track += sizeof(TaskQue);

    que->front_content = NULL;
    que->rear_content = NULL;
    que->size = 0;
    que->leaked_bytes = 0;
    return que;
}

/**
 * Push the task in task_que
 * @param que       : Pointer to task_que where content is to be added
 * @param task      : Task to be pushed
 * @return          : Pointer to same task_que
 *                  : NULL if heap is full
 */
TaskQue *task_que_push(TaskQue *que, Task task) {
    if(task_que_isExist(que,task))
        return que;
    TaskQueContent *content = (TaskQueContent *) malloc(sizeof(TaskQueContent));
    if(content==NULL)
        return NULL;
    int size = sizeof(TaskQueContent);
    task_que_leak_track += size;
    que->leaked_bytes += size;

    content->id = task_que_id_count++;
    content->task = task;
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
 * Return the first value in task_que
 * @param que       : TaskQue
 * @return          : First task in task_que
 *                  : NULL is task_que is empty
 */
Task task_que_get(TaskQue* que){
    if((que->size)<=0)
        return NULL;
    return que->front_content->task;
}

/**
 * Remove the first value of task_que
 * @param que       : Pointer to the task_que
 * @return          : Pointer to same task_que
 */
TaskQue *task_que_pop(TaskQue *que) {
    if (que->size <= 0)
        return que;

    TaskQueContent *temp = que->front_content;
    que->front_content = temp->next_content;

    free((TaskQueContent *) temp);
    int size = sizeof(TaskQueContent);
    task_que_leak_track -= size;
    que->leaked_bytes -= size;

    que->size--;
    if (que->size == 0) {
        que->front_content = NULL;
        que->rear_content = NULL;
    }
    return que;
}

/**
 * Delete the whole task_que
 * @param que       : Pointer to task_que
 */
void task_que_delete(TaskQue *que) {
    if (que == NULL)
        return;
    int len = que->size;
    for (int i = 0; i < len; i++)
        task_que_pop(que);
    free(que);
    task_que_leak_track -= sizeof(TaskQue);
}

/**
 * Print the task_que
 * @param que       : TaskQue
 * @param print     : callback combiner_test_print function (i.e printf)
 */
void task_que_print(TaskQue* que, int (*print)(const char *format, ...)) {
    TaskQueContent *content = que->front_content;
    print("[ ");
    for (int i = 0; i < que->size; ++i) {
        print("%d ", content->id);
        content = content->next_content;
    }
    print("]\n");
}

/**
 * This check if task exist or not
 * @param que   : Que
 * @param task  : Task ot be compared
 * @return      : 1 for exist
 *              : 0 for doesn't exist
 */
int task_que_isExist(TaskQue* que, Task task){
    int len = que->size;
    TaskQueContent *temp = que->front_content;
    for(int i = 0; i < len; i++){
        if(temp->task==task)
            return 1;
        temp = temp->next_content;
    }
    return 0;
}

/**
 * Return the leaked bytes
 * @param que       : Task task_que or NULL
 * return 			: Leaked bytes
 */
int task_que_getLeakedBytes(TaskQue* que){
    if(que==NULL)
        return task_que_leak_track;
    return que->leaked_bytes;
}
