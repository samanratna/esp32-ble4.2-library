//
// Created by peter on 12/24/2022.
//

#ifndef COMMUNICATION_MAP_H
#define COMMUNICATION_MAP_H

#include "stdint.h"

//Store any kind of data
typedef void *MapValue;
typedef uint32_t MapKey;

//It is content of Map
struct MapContent {
    int id;
    MapKey key;
    MapValue value;
    struct MapContent *next_content;
};
typedef struct MapContent MapContent;

//It is task_que
typedef struct {
    MapContent *front_content;
    MapContent *rear_content;
    int size;
    int leaked_bytes;
} Map;

Map *map_create();

void map_getKeys(Map *, MapKey *);

MapValue map_get(Map *, MapKey);

Map *map_put(Map *, MapKey, MapValue);

Map *map_remove(Map *, MapKey);

void map_delete(Map *);

void map_print(Map *, int (*)(const char *format, ...));

int map_getLeakedBytes(Map *);

void map_test();

#endif //COMMUNICATION_MAP_H
