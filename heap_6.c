/*
    FreeRTOS V8.0.1 - Copyright (C) 2014 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */
#include <stdlib.h>
#include <stdbool.h>
void * memmove ( void * destination, const void * source, size_t num );

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( heapSTRUCT_SIZE * 2 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE		( ( size_t ) 8 )

/* A few bytes might be lost to byte aligning the heap start address. */
#define heapADJUSTED_HEAP_SIZE	( configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT )

#define HEAP_MAGIC 0xabcddcba
#define NO_GC_MAGIC ((void*)((uint)-1))
#define heapMAGIC_SIZE 4

/* Allocate the memory for the heap. */
#include "heap_regions.h"

typedef struct HeapRegion
{
	uint8_t *pucStartAddress; // << Start address of a block of memory that will be part of the heap.
	size_t xSizeInBytes;	 // << Size of the block of memory.
} HeapRegion_t;


/* Define the structure.  This is used to link free blocks in order
of their memory address, and as a tree structure for allocated blocks */
typedef struct garbage_node {
    struct garbage_node* prev;
    struct garbage_node* next;
    size_t size;
    void (*dest)(void*);
} garbage_node;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void prvInsertBlockIntoFreeList( garbage_node *pxBlockToInsert );

/*
 * Called automatically to setup the required heap structures the first time
 * pvPortMalloc() is called.
 */
static void prvHeapInit( void );

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const uint16_t heapSTRUCT_SIZE	= ( ( sizeof ( garbage_node ) ) );

/* Ensure the pxEnd pointer will end up on the correct byte alignment. */
//static const size_t xTotalHeapSize = ( ( size_t ) heapADJUSTED_HEAP_SIZE ) & ( ( size_t ) ~portBYTE_ALIGNMENT_MASK );

/* Create a couple of list links to mark the start and end of the list. */
static garbage_node xStart, *pxEnd = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t xFreeBytesRemaining = ( ( size_t ) heapADJUSTED_HEAP_SIZE ) & ( ( size_t ) ~portBYTE_ALIGNMENT_MASK );
static size_t xMinimumEverFreeBytesRemaining = ( ( size_t ) heapADJUSTED_HEAP_SIZE ) & ( ( size_t ) ~portBYTE_ALIGNMENT_MASK );


void *pvPortMallocGC( size_t xWantedSize );
void *pvPortMalloc( size_t xWantedSize );


static void raw_free( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
garbage_node *pxLink;

	if( pv != NULL )
	{
		/* The memory being freed will have an garbage_node structure immediately
		before it. */
		puc -= heapSTRUCT_SIZE;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = ( void * ) puc;

		/* Check the block is actually allocated. */
		configASSERT( *(int*)(puc+pxLink->size-4) == HEAP_MAGIC );

        vTaskSuspendAll();
        {
            /* Add this block to the list of free blocks. */
            xFreeBytesRemaining += pxLink->size;
            traceFREE( pv, pxLink->size );
            prvInsertBlockIntoFreeList( ( ( garbage_node * ) pxLink ) );
        }
        ( void ) xTaskResumeAll();
	}
}

/*-----------------------------------------------------------*/
void *pvPortRealloc( void *pv, size_t xWantedSize )
{
uint8_t *puc = ( uint8_t * ) pv;
garbage_node *pxLink, *pxNewBlockLink;

	if( xWantedSize == 0 ) {
		raw_free(pv);
		return NULL;
	}

	if( pv != NULL )
	{
		vTaskSuspendAll();

		/* The memory will have an garbage_node structure immediately
		before it. */
		puc -= heapSTRUCT_SIZE;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = ( void * ) puc;

		/* Check the block is actually allocated. */
		configASSERT( *(int*)(puc+pxLink->size)-4 == HEAP_MAGIC );

		xWantedSize += heapSTRUCT_SIZE;
		xWantedSize += heapMAGIC_SIZE;

		/* Ensure that blocks are always aligned to the required number
		of bytes. */
		if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
		{
			/* Byte alignment required. */
			xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		if( xWantedSize == pxLink->size ) {
			( void ) xTaskResumeAll();
			return pv;
		} else if( xWantedSize < pxLink->size ) { //realloc'ing down (or maybe reallocing up but was given a really large block to start, consider tracking used size as well as block size)
			if(xWantedSize < heapMINIMUM_BLOCK_SIZE || pxLink->size - xWantedSize < heapMINIMUM_BLOCK_SIZE) {
				//don't change anything - the block is already too small and there's not enough space being freed to make a new block
				( void ) xTaskResumeAll();
				return pv;
			}

			size_t diff =  pxLink->size - xWantedSize;

			//split the block and insert the new one into the free list
			pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxLink ) + xWantedSize );

			/* Calculate the sizes of two blocks split from the
			single block. */
			pxNewBlockLink->size = diff;
			xFreeBytesRemaining += diff;
			pxLink->size = xWantedSize;
			*(int*)(puc+xWantedSize-4) = HEAP_MAGIC;

			/* Insert the new block into the list of free blocks. */
			prvInsertBlockIntoFreeList( ( pxNewBlockLink ) );

			traceFREE( pv, diff );
			( void ) xTaskResumeAll();
			return pv;
		} else {
			//make sure it's not too much...
			if( puc + xWantedSize > (uint8_t*)pxEnd ) {
				( void ) xTaskResumeAll();
				return NULL;
			}
			size_t original_size = pxLink->size;
			size_t diff = xWantedSize - original_size;

			//increasing size...
			pxNewBlockLink = (garbage_node*)(puc+pxLink->size);
			if( !pxNewBlockLink->size
					&& xWantedSize<pxLink->size+pxNewBlockLink->size ) //see if we can grab the next block...
			{
				/*  Walk the free list to find the previous link so we can remove the next block from the list */
				garbage_node * pxPreviousBlock = &xStart;
				garbage_node * pxBlock = xStart.next;
				garbage_node * pxDest = (garbage_node*)(puc+xWantedSize);

				while( ( pxBlock !=  pxNewBlockLink) && ( pxBlock->next != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->next;
				}
				*pxDest = *pxNewBlockLink;
				pxDest->size -= diff;
				pxPreviousBlock->next = pxDest;
				pxLink->size = xWantedSize;
				*(int*)(puc+xWantedSize-4) = HEAP_MAGIC;

				xFreeBytesRemaining -= diff;

				traceMALLOC( pv, diff );
				( void ) xTaskResumeAll();
				return pv;
			}
			//got to memmove... didn't find enough contiguous blocks
			{
				void * mem;
                if( pxLink->next == NO_GC_MAGIC ) {
                    mem = pvPortMalloc(xWantedSize);
                } else {
                    mem = pvPortMallocGC(xWantedSize);
                }
				if( mem ) {
					memmove(mem, pv, pxLink->size - heapSTRUCT_SIZE);
					raw_free(pv);
					( void ) xTaskResumeAll();
					return mem;
				}
				( void ) xTaskResumeAll();
				return NULL;
			}
		}
	} else {
		return pvPortMalloc(xWantedSize);
	}
}

/*-----------------------------------------------------------*/

static void *raw_malloc( size_t xWantedSize )
{
garbage_node *pxBlock, *pxPreviousBlock, *pxNewBlockLink;
void *pvReturn = NULL;

	{
		/* If this is the first call to malloc then the heap will require
		initialisation to setup the list of free blocks. */
		if( pxEnd == NULL )
		{
			prvHeapInit();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

        /* The wanted size is increased so it can contain a garbage_node
        structure in addition to the requested amount of bytes. */
        if( xWantedSize > 0 )
        {
            xWantedSize += heapSTRUCT_SIZE;
            xWantedSize += heapMAGIC_SIZE;

            /* Ensure that blocks are always aligned to the required number
            of bytes. */
            if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
            {
                /* Byte alignment required. */
                xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        if( ( xWantedSize > 0 ) && ( xWantedSize <= xFreeBytesRemaining ) )
        {
            /* Traverse the list from the start	(lowest address) block until
            one	of adequate size is found. */
            pxPreviousBlock = &xStart;
            pxBlock = xStart.next;
            while( ( pxBlock->size < xWantedSize ) && ( pxBlock->next != NULL ) )
            {
                pxPreviousBlock = pxBlock;
                pxBlock = pxBlock->next;
            }

            /* If the end marker was reached then a block of adequate size
            was	not found. */
            if( pxBlock != pxEnd )
            {
                /* Return the memory space pointed to - jumping over the
                garbage_node structure at its start. */
                pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->next ) + heapSTRUCT_SIZE );

                /* This block is being returned for use so must be taken out
                of the list of free blocks. */
                pxPreviousBlock->next = pxBlock->next;

                /* If the block is larger than required it can be split into
                two. */
                if( ( pxBlock->size - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
                {
                    /* This block is to be split into two.  Create a new
                    block following the number of bytes requested. The void
                    cast is used to prevent byte alignment warnings from the
                    compiler. */
                    pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );

                    /* Calculate the sizes of two blocks split from the
                    single block. */
                    pxNewBlockLink->size = pxBlock->size - xWantedSize;
                    pxBlock->size = xWantedSize;

                    /* Insert the new block into the list of free blocks. */
                    prvInsertBlockIntoFreeList( ( pxNewBlockLink ) );
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                xFreeBytesRemaining -= pxBlock->size;

                if( xFreeBytesRemaining < xMinimumEverFreeBytesRemaining )
                {
                    xMinimumEverFreeBytesRemaining = xFreeBytesRemaining;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }

                /* The block is being returned - it is allocated and owned
                by the application and has no "next" block. */
                *(int*)(((char*)pxBlock)+pxBlock->size-4) = HEAP_MAGIC;
                pxBlock->next = pxBlock->prev = NO_GC_MAGIC;
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

		traceMALLOC( pvReturn, xWantedSize );
	}
    
    #if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	return pvReturn;
}



static garbage_node* garbage_splay_(garbage_node * root, const void* addr) {
    // This is a variant of Daniel Sleator's public domain top-down
    // splay function.  It has been changed to handle address ranges.
    if (!root) return 0;

    garbage_node node = {};
    garbage_node* head = &node;
    garbage_node* tail = &node;

    for (;;) {
        if ((uintptr_t)addr < (uintptr_t)root) {
            garbage_node* prev = root->prev;
            if (!prev) break;
            if ((uintptr_t)addr < (uintptr_t)prev) {
                root->prev = prev->next;
                prev->next = root;
                root = prev;
                if (!root->prev) break;
            }
            tail->prev = root;
            tail = root;
            root = root->prev;
            continue;
        }
        if ((uintptr_t)addr >= (uintptr_t)(root+root->size)) {
            garbage_node* next = root->next;
            if (!next) break;
            if ((uintptr_t)addr >= (uintptr_t)(next+next->size)) {
                root->next = next->prev;
                next->prev = root;
                root = next;
                if (!root->next) break;
            }
            head->next = root;
            head = root;
            root = root->next;
            continue;
        }
        break;
    }

    head->next = root->prev;
    tail->prev = root->next;
    root->prev = node.next;
    root->next = node.prev;
    return root;
}


/*-----------------------------------------------------------*/
typedef struct garbage_task {
    struct garbage_task * next;
    bool collected;
} garbage_task;
typedef struct {
    garbage_node * working;
    garbage_node * grey;
    garbage_node * black;
    garbage_task * tasks;
} garbage_root;

static garbage_node* garbage_insert_(garbage_node* root, garbage_node* node) {
    if(!node) return root;
    if (!root) {
        node->next = 0;
        node->prev = 0;
        return node;
    }

    root = garbage_splay_(root, node);
    if ((uintptr_t)node > (uintptr_t)root) {
        node->next = root->next;
        node->prev = root;
        root->next = 0;
    } else {
        node->next = root;
        node->prev = root->prev;
        root->prev = 0;
    }

    return node;
}
static garbage_node* garbage_find_( garbage_node * heap, garbage_node * grey ) {
    while (grey) {
        grey = garbage_splay_(grey, 0);
        garbage_node *node = grey;
        grey = grey->next;
        heap = garbage_insert_(heap, node );
    }
    return heap;
}

static void garbage_destruct_(garbage_node* node) {
    if (node->dest) {
        node->dest(node + 1);
    }
    raw_free(node+1);
}

static garbage_node* garbage_remove_(garbage_node ** root, const void* addr) {
    if (!root||!(*root)) return 0;
    *root = garbage_splay_((*root), addr);
    if ((uintptr_t)addr < (uintptr_t)((*root) + 1)) return 0;
    if ((uintptr_t)addr >= (uintptr_t)((*root)+(*root)->size)) return 0;

    garbage_node* node = 0;
    if ((*root)->next) {
        garbage_node* next = garbage_splay_((*root)->next, 0);
        next->prev = (*root)->prev;
        node = (*root);
        (*root) = next;
    } else {
        node = (*root);
        (*root) = (*root)->prev;
    }
    node->next = 0;
    node->prev = 0;
    return node;
}

static void* garbage_object_map(garbage_root * groot, void * data, ssize_t bytes, void (*destruct)(void*)) {
    garbage_node* node = data - sizeof(garbage_node);
    node->dest = destruct;
    (groot->working) = garbage_insert_(groot->working, node);
    return node + 1;
}

static garbage_node * garbage_lose_( garbage_node ** root, garbage_node * grey) {
    garbage_node* black = 0;
    while (grey) {
        grey = garbage_splay_(grey, 0);
        garbage_node *node = grey;
        grey = grey->next;
        if (node->dest) {
            const void** nodelo = (const void**)((char*)(node + 1)+1);
            const void** nodehi = (const void**)(node+node->size);
            for (const void** ptr = nodelo; ptr < nodehi; ptr++) {
                garbage_node* next = garbage_remove_(root, *ptr);
                if (next) {
                    grey = garbage_insert_(grey, next);
                }
            }
        }
        //all the children of the one have been added to the grey set, it can now be moved to the black set
        black = garbage_insert_(black, node);
    }
    return black;
}
static void garbage_cleanup(garbage_root * groot) {
    while ((groot->working)) {
        if ((groot->working)->prev) (groot->working) = garbage_splay_((groot->working), 0);
        garbage_node* node = (groot->working);
        (groot->working) = (groot->working)->next;
        garbage_destruct_(node);
    }
    while ((groot->grey)) {
        if ((groot->grey)->prev) (groot->grey) = garbage_splay_((groot->grey), 0);
        garbage_node* node = (groot->grey);
        (groot->grey) = (groot->grey)->next;
        garbage_destruct_(node);
    }
}
#include <setjmp.h>
static void garbage_collect(garbage_root * groot, void * stk ) {
    jmp_buf jb;
    setjmp(jb); //dump registers to stack

    //grey set are not fully explored
    //take our set of allocations, move the ones referenced in the stack to the grey set
    garbage_node* referenced = 0;
    const void** stacklo = (const void**)&jb;
    const void** stackhi = stk;
    for (const void** ptr = stacklo; ptr < stackhi; ptr++) {
        referenced = garbage_insert_(referenced, garbage_remove_(&groot->grey, *ptr));
        referenced = garbage_insert_(referenced, garbage_remove_(&groot->working, *ptr));
    }
    //drop all the referenced allocations from the working set into the black set
    groot->black = garbage_find_( groot->black, garbage_lose_(&groot->working, referenced));

    garbage_task * tasks = groot->tasks;
    bool all_collected = true;
    while(tasks!=NULL) {
        if(tasks==stk) {
            tasks->collected = true;
        }
        if( !tasks->collected ) {
            all_collected = false;
        }
        tasks = tasks->next;
    }
    //make sure we scan all the stacks before sweeping up
    if( all_collected) {
        //anything left in the grey set wasn't referenced by any new objects or stacks and may be collected
        while (groot->grey) {
            if (groot->grey->prev) groot->grey = garbage_splay_(groot->grey, 0);
            garbage_node* node = groot->grey;
            groot->grey = groot->grey->next;
            garbage_destruct_(node);
        }
        //working set is now frozen until all tasks can mark it
        groot->grey = groot->working;
        //new allocations can be done in black set
        groot->working = groot->black;
        groot->black = 0;
        
        tasks = groot->tasks;
        while(tasks!=NULL) {
            tasks->collected = false;
            tasks = tasks->next;
        }
    }
}
static garbage_root heap_groot={0};


void *pvPortMalloc( size_t xWantedSize ) {
	void * p;
    vTaskSuspendAll();
    p = raw_malloc( xWantedSize);
    ( void ) xTaskResumeAll();
    return p;
}
void vPortFree( void *pv ) {
    vTaskSuspendAll();
    raw_free(pv);
    ( void ) xTaskResumeAll();
}

static void garbage_noop_(void* ignored) {}
void *pvPortMallocGC( size_t xWantedSize ) {
	void * p;
    vTaskSuspendAll();
    p = garbage_object_map( &heap_groot, raw_malloc(xWantedSize), xWantedSize, garbage_noop_);
    ( void ) xTaskResumeAll();
    return p;
}
void *pvPortMallocGCObject( size_t xWantedSize, void (*destruct)(void*) ) {
    void * p;
    vTaskSuspendAll();
    p = garbage_object_map( &heap_groot, raw_malloc(xWantedSize), xWantedSize, destruct);
    ( void ) xTaskResumeAll();
    return p;
}
void *pvPortMallocGCBuffer(  size_t xWantedSize ) {
	void * p;
    vTaskSuspendAll();
    p = garbage_object_map( &heap_groot, raw_malloc(xWantedSize), xWantedSize, NULL);
    ( void ) xTaskResumeAll();
    return p;
}
void pvPortGarbageRegister(void * stk) {
    vTaskSuspendAll();
    garbage_task * this = stk;
    this->collected = false;
    this->next = heap_groot.tasks;
    heap_groot.tasks = this;
    ( void ) xTaskResumeAll();
}
void pvPortGarbageUnregister(void * stk) {
    vTaskSuspendAll();
    garbage_task * tasks = heap_groot.tasks;
    garbage_task * prev = NULL;
    while(tasks!=NULL) {
        if(tasks==stk) {
            if( prev ) {
                prev->next = tasks->next;
            } else {
                heap_groot.tasks = NULL;
            }
            break;
        }
        prev = tasks;
        tasks = tasks->next;
    }
    ( void ) xTaskResumeAll();
}
void pvPortGarbageCollect(void * stk) {
    vTaskSuspendAll();
    garbage_collect(&heap_groot, stk);
    ( void ) xTaskResumeAll();
}
void pvPortGarbageCleanup() {
    vTaskSuspendAll();
    garbage_cleanup(&heap_groot);
    ( void ) xTaskResumeAll();
}
void vPortFreeGC( void *pv ) {
    vTaskSuspendAll();
    raw_free(pv);
    garbage_node *node = garbage_remove_(&heap_groot.working, pv);
    if (node) {
         garbage_destruct_(node);
    } else if((node = garbage_remove_(&heap_groot.grey, pv))) {
         garbage_destruct_(node);
    } else if((node = garbage_remove_(&heap_groot.black, pv))) {
         garbage_destruct_(node);
    }
    ( void ) xTaskResumeAll();
}

/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void )
{
	return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t xPortGetMinimumEverFreeHeapSize( void )
{
	return xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
	/* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

void vPortDefineHeapRegions( const HeapRegion_t * const pxHeapRegions )
{
garbage_node *pxFirstFreeBlockInRegion = NULL, *pxPreviousFreeBlock;
uint8_t *pucAlignedHeap;
size_t xTotalRegionSize, xTotalHeapSize = 0;
BaseType_t xDefinedRegions = 0;
size_t ulAddress;
const HeapRegion_t *pxHeapRegion;

	/* Can only call once! */
	configASSERT( pxEnd == NULL );

	pxHeapRegion = &( pxHeapRegions[ xDefinedRegions ] );

	while( pxHeapRegion->xSizeInBytes > 0 )
	{
		xTotalRegionSize = pxHeapRegion->xSizeInBytes;

		/* Ensure the heap region starts on a correctly aligned boundary. */
		ulAddress = ( size_t ) pxHeapRegion->pucStartAddress;
		if( ( ulAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
		{
			ulAddress += ( portBYTE_ALIGNMENT - 1 );
			ulAddress &= ~portBYTE_ALIGNMENT_MASK;

			/* Adjust the size for the bytes lost to alignment. */
			xTotalRegionSize -= ulAddress - ( size_t ) pxHeapRegion->pucStartAddress;
		}

		pucAlignedHeap = ( uint8_t * ) (void*) ulAddress;

		/* Set xStart if it has not already been set. */
		if( xDefinedRegions == 0 )
		{
			/* xStart is used to hold a pointer to the first item in the list of
			free blocks.  The void cast is used to prevent compiler warnings. */
			xStart.next = ( garbage_node * ) pucAlignedHeap;
			xStart.size = ( size_t ) 0;
		}
		else
		{
			/* Should only get here if one region has already been added to the
			heap. */
			configASSERT( pxEnd != NULL );

			/* Check blocks are passed in with increasing start addresses. */
			configASSERT( ulAddress > ( size_t ) pxEnd );
		}

		/* Remember the location of the end marker in the previous region, if
		any. */
		pxPreviousFreeBlock = pxEnd;

		/* pxEnd is used to mark the end of the list of free blocks and is
		inserted at the end of the region space. */
		ulAddress = ( ( size_t ) pucAlignedHeap ) + xTotalRegionSize;
		ulAddress -= heapSTRUCT_SIZE;
		ulAddress &= ~portBYTE_ALIGNMENT_MASK;
		pxEnd = ( garbage_node * ) ulAddress;
		pxEnd->size = 0;

		/* To start with there is a single free block in this region that is
		sized to take up the entire heap region minus the space taken by the
		free block structure. */
		pxFirstFreeBlockInRegion = ( garbage_node * ) pucAlignedHeap;
		pxFirstFreeBlockInRegion->size = ulAddress - ( size_t ) pxFirstFreeBlockInRegion;
		pxFirstFreeBlockInRegion->next = pxEnd;

		/* If this is not the first region that makes up the entire heap space
		then link the previous region to this region. */
		if( pxPreviousFreeBlock != NULL )
		{
			pxPreviousFreeBlock->next = pxFirstFreeBlockInRegion;
		}

		xTotalHeapSize += pxFirstFreeBlockInRegion->size;

		/* Move onto the next HeapRegion_t structure. */
		xDefinedRegions++;
		pxHeapRegion = &( pxHeapRegions[ xDefinedRegions ] );
	}

	xMinimumEverFreeBytesRemaining = xTotalHeapSize;
	xFreeBytesRemaining = xTotalHeapSize;

	/* Check something was actually defined before it is accessed. */
	configASSERT( xTotalHeapSize );
}

static void prvHeapInit( void )
{
	HeapRegion_t xHeapRegions[NUM_HEAP_REGIONS+1];

	heap_regions_init(xHeapRegions);

	vPortDefineHeapRegions( xHeapRegions );
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( garbage_node *pxBlockToInsert )
{
garbage_node *pxIterator;
uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	than the block being inserted. */
	for( pxIterator = &xStart; pxIterator->next < pxBlockToInsert; pxIterator = pxIterator->next )
	{
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->size ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->size += pxBlockToInsert->size;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->size ) == ( uint8_t * ) pxIterator->next )
	{
		if( pxIterator->next != pxEnd )
		{
			/* Form one big block from the two blocks. */
			pxBlockToInsert->size += pxIterator->next->size;
			pxBlockToInsert->next = pxIterator->next->next;
		}
		else
		{
			pxBlockToInsert->next = pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->next = pxIterator->next;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	before and the block after, then it's next pointer will have
	already been set, and should not be set here as that would make it point
	to itself. */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->next = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}

