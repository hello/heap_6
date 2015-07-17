heap_6
======

An extension of heap_5.c which allows for heap visualization (helps determine if it's fragementing) and efficient reallocation.

Requirements
============

Expects a heap_regions.c in the include path, here is an example of the content:

	__attribute__((section(".data"))) static uint8_t ucHeap_DATA[ configTOTAL_HEAP_SIZE_DATA ];
	__attribute__((section(".bss"))) static uint8_t ucHeap_BSS[ configTOTAL_HEAP_SIZE_BSS ];

	#define heap_regions_init(xHeapRegions) \
	xHeapRegions[0].pucStartAddress = ucHeap_BSS; \
	xHeapRegions[0].xSizeInBytes =  sizeof(ucHeap_BSS); \
	xHeapRegions[1].pucStartAddress = ucHeap_DATA; \
	xHeapRegions[1].xSizeInBytes =  sizeof(ucHeap_DATA); \
	xHeapRegions[2].pucStartAddress = NULL; \
	xHeapRegions[2].xSizeInBytes =  0;

	#define NUM_HEAP_REGIONS 2

