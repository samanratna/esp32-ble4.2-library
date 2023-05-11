//
// Created by peter on 12/13/2022.
//

#ifndef PETER_PRACTICE_C_QUE_H
#define PETER_PRACTICE_C_QUE_H

typedef void* QueType;

//It is content of Que
struct QueContent{
    int id;
    QueType value;
    struct QueContent *next_content;
};
typedef struct QueContent QueContent;

typedef struct {
    QueContent *front_content;
    QueContent *rear_content;
    int size;
    int leaked_bytes;
} Que;

Que *que_create();
Que *que_push(Que*, QueType);
QueType que_get(Que*);
Que *que_pop(Que*);
void que_delete(Que*);
void que_print(Que*, int (*)(const char *format, ...));
int que_isExist(Que*, QueType);
int que_getLeakedBytes(Que*);

void que_test();


#endif //PETER_PRACTICE_C_QUE_H