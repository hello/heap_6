#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <pthread.h>

void *pvPortRealloc( void *pv, size_t xWantedSize );
void *pvPortMallocGC( size_t xWantedSize );
void *pvPortMallocGCObject( size_t xWantedSize, void (*destruct)(void*) );
void vPortFree( void *pv );

#define GC_STACK (2*sizeof(void*))
void pvPortGarbageRegister(void * stk);
void pvPortGarbageUnregister(void * stk);
void pvPortGarbageCollect(void * stk);
void pvPortGarbageCleanup();

size_t xPortGetFreeHeapSize( void );


static int count = 0;
static void destroy (void* index) {
    count--;
   // printf("destroy: %d\n", *(char*)index);
}

static  void dbg_test(void * stk) {

    char* scratch[10] = {};
    
    for (int i = 0; i<200; i++) {
        ssize_t index = rand();
        index = (index ^ (index>>8)) % 10;
        
        scratch[index] = pvPortMallocGCObject(1, destroy);
        *scratch[index] = index;
        int start_count = ++count;
        pvPortGarbageCollect(stk);
        printf("collect: %d %d %ld %lx %ld\n", start_count, count, xPortGetFreeHeapSize(), (long)stk, (long)index);
    }

    printf( "after loop: %d %ld\n", count, xPortGetFreeHeapSize());}

void *garbage_test(void *threadid)
{
    char my_stk[GC_STACK];
    pvPortGarbageRegister(&my_stk);
    dbg_test(&my_stk);
    pvPortGarbageUnregister(&my_stk);
    pthread_exit(NULL);
}

extern pthread_mutex_t m;

#define NUM_THREADS 5
int main (int argc, char* argv[]) {
    
    void *status;
    pthread_t threads[NUM_THREADS];
    pthread_mutexattr_t a;

    pthread_mutexattr_init(&a);
    pthread_mutexattr_settype(&a, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&m, &a);

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    int rc;
    long t;
    for(t=0; t<NUM_THREADS; t++){
      printf("In main: creating thread %ld\n", t);
      rc = pthread_create(&threads[t], &attr, garbage_test, (void *)t);
      if (rc){
         printf("ERROR; return code from pthread_create() is %d\n", rc);
         exit(-1);
      }
    }
    

   for(t=0; t<NUM_THREADS; t++) {
        pthread_join(threads[t], &status);
    }
    pvPortGarbageCleanup();
    printf( "after cleanup: %d %ld\n", count, xPortGetFreeHeapSize());
    
    pthread_exit(NULL);
    return 0;
}
