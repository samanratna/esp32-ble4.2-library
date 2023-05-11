//
// Created by peter on 12/24/2022.
//

#include "map.h"
#include "stdlib.h"

int map_leak_track = 0;
static int map_id_count = 0;

/**
 * Create task_que and return pointer to it
 * @return          : Pointer to map (!!!dynamically allocated : map_delete(map) must be called)
 *                  : NULL if heap is full
 */
Map *map_create() {
    Map *map = (Map *) malloc(sizeof(Map));
    if(map==NULL)
        return NULL;
    map_leak_track += sizeof(Map);

    map->front_content = NULL;
    map->rear_content = NULL;
    map->size = 0;
    map->leaked_bytes = 0;
    return map;
}

/**
 * Return all keys
 * @param map       : pointer to map
 * @param keys      : get all keys
 */
void map_getKeys(Map *map,MapKey* keys){
    MapContent* content = map->front_content;
    for (int i = 0; i < map->size; ++i) {
   	    if(content==NULL)
   		   return;
        keys[i] = content->key;
        content = content->next_content;
    }
}

/**
 * Return the value corresponding to key
 * @param map       : Map
 * @param key       : Key corresponding to value
 * @return          : Value corresponding to key
 *                  : NULL is key doesn't exist
 */
MapValue map_get(Map *map, MapKey key) {
    int len = map->size;
    MapContent *content = map->front_content;
    for (int i = 0; i < len; i++) {
        if (content->key == key)
            return content->value;
        content = content->next_content;
    }
    return NULL;
}


/**
 * Put the value in map
 * @param map       : Pointer to map where content is to be added
 * @param key       : key of map
 * @param value     : Value to be put
 * @return          : Pointer to same map
 *                  : NULL if heap is full
 */
Map *map_put(Map *map, MapKey key, MapValue value) {
    MapContent *content = map_get(map, key);
    if (content == NULL) {
        int size = sizeof(MapContent);
        content = (MapContent *) malloc(size);
        if(content==NULL)
            return NULL;
        map->leaked_bytes += size;
        map_leak_track += size;
    }

    content->key = key;
    content->id = map_id_count++;
    content->value = value;
    content->next_content = NULL;

    if (map->size == 0)
        map->front_content = content;
    else
        map->rear_content->next_content = content;

    map->rear_content = content;
    map->size++;

    return map;
}


/**
 * Remove the content corresponding to key from map
 * @param map       : Pointer to the map
 * @param key       : Key of content
 * @return          : Pointer to same map
 *                  : NULL if key doesn't exist
 */
Map *map_remove(Map *map, MapKey key) {

    if (map->size <= 0)
        return map;
    MapContent *front = NULL;
    MapContent *content = map->front_content;

    int i;
    for (i = 0; i < map->size; i++) {
        if (key == content->key)
            break;
        front = content;
        content = content->next_content;
    }
    if(i==map->size)
        return NULL;

    if(front==NULL)
        map->front_content = content->next_content;
    else
        front->next_content = content->next_content;
    free((MapContent *) content);

    int size = sizeof(MapContent);
    map->leaked_bytes -= size;
    map_leak_track -= size;

    map->size--;
    if (map->size == 0) {
        map->front_content = NULL;
        map->rear_content = NULL;
    }
    return map;
}

/**
 * Delete the whole map
 * @param map       : Pointer to map
 */
void map_delete(Map *map) {
    if (map == NULL)
        return;
    int len = map->size;
    MapContent *content = map->front_content;
    MapContent *nextContent;
    for (int i = 0; i < len; i++) {
        if (content == NULL)
            break;
        nextContent = content->next_content;
        free((MapContent *) content);
        int size = sizeof(MapContent);
        map_leak_track -= size;
        map->leaked_bytes -=size;
        content = nextContent;
    }
    free(map);
    map_leak_track -= sizeof(Map);
}

/**
 * Print the map
 * @param map       : Map
 * @param print     : callback combiner_test_print function (i.e printf)
 */
void map_print(Map *map, int (*print)(const char *format, ...)) {
    MapContent *content = map->front_content;
    print("[ ");
    for (int i = 0; i < map->size; ++i) {
        print("%d ", content->id);
        content = content->next_content;
    }
    print("]\n");
}

/**
 * Return the leaked bytes
 * @param map       : Pointer to map
 * return 			: Leaked bytes (NULL for all)
 */
int map_getLeakedBytes(Map *map) {
    if (map == NULL)
        return map_leak_track;
    return map->leaked_bytes;
}
