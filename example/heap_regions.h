#define configTOTAL_HEAP_SIZE 10000

static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

#define heap_regions_init(xHeapRegions) \
xHeapRegions[0].pucStartAddress = ucHeap; \
xHeapRegions[0].xSizeInBytes =  sizeof(ucHeap); \
xHeapRegions[1].pucStartAddress = NULL; \
xHeapRegions[1].xSizeInBytes =  0;

#define NUM_HEAP_REGIONS 1


#define portBYTE_ALIGNMENT_MASK 0x7
#define portBYTE_ALIGNMENT 8

void vPortFree( void *pv );

#define configASSERT(...)
#define traceFREE(...)
#define traceMALLOC(...)
#define mtCOVERAGE_TEST_MARKER(...)


typedef long BaseType_t;

#include <pthread.h>
#include <stdbool.h>
pthread_mutex_t m = {0};
bool mut_init=false;
static void vTaskSuspendAll(void){
    pthread_mutex_lock (&m);
}
static long xTaskResumeAll(void){
    pthread_mutex_unlock (&m);
    return 0;
}
