#ifndef QUEUE_H_
#define QUEUE_H_


#include "stdlib.h"
#include "stdbool.h"

typedef struct
{
    int front;
    int rear;
    int used;
    unsigned capacity;
    int *array;
}queue_t;

queue_t *create_queue(int capacity);
int getFront(queue_t *queue);

int enqueue(queue_t *queue, int item);
int dequeue(queue_t *queue, int *item);

#endif

