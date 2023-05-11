//
// Created by peter on 12/13/2022.
//

#ifndef PETER_PRACTICE_C_TASK_QUE_H
#define PETER_PRACTICE_C_TASK_QUE_H

//Store function of type int func(int len,...)
typedef int (*Task)();

//It is content of TaskQue
struct TaskQueContent{
    int id;
    Task task;
    struct TaskQueContent *next_content;
};
typedef struct TaskQueContent TaskQueContent;

//It is task_que
typedef struct {
    TaskQueContent *front_content;
    TaskQueContent *rear_content;
    int size;
    int leaked_bytes;
} TaskQue;

TaskQue *task_que_create();
TaskQue *task_que_push(TaskQue*, Task);
Task task_que_get(TaskQue*);
TaskQue *task_que_pop(TaskQue*);
void task_que_delete(TaskQue*);
int task_que_getLeakedBytes(TaskQue*);
void task_que_print(TaskQue*, int (*)(const char *format, ...));
int task_que_isExist(TaskQue*, Task);

void task_que_test();


#endif //PETER_PRACTICE_C_TASK_QUE_H