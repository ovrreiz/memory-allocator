// SPDX-License-Identifier: BSD-3-Clause

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>

#include "osmem.h"
#include "block_meta.h"
#if defined(__has_include)
#if __has_include("helpers.h")
#include "helpers.h"
#endif
#endif

#ifndef DIE
#include <stdio.h>
#include <stdlib.h>
#define DIE(assertion, call_description)                                  \
	do {                                                                  \
		if (assertion) {                                                  \
			fprintf(stderr, "(%s, %d): ", __FILE__, __LINE__);            \
			perror(call_description);                                     \
			exit(errno);                                                  \
		}                                                                 \
	} while (0)
#endif

#ifndef STATUS_FREE
#define STATUS_FREE   0
#define STATUS_ALLOC  1
#define STATUS_MAPPED 2
#endif

#ifndef MMAP_THRESHOLD
#define MMAP_THRESHOLD (128 * 1024)
#endif

#define ALIGNMENT 8
#define ALIGN(x) (((x) + (ALIGNMENT - 1)) & ~((size_t)ALIGNMENT - 1))
#define META_SIZE ALIGN(sizeof(struct block_meta))
#define BLOCK_OF(ptr) ((struct block_meta *)((char *)(ptr) - META_SIZE))
#define PAYLOAD_OF(blk) ((void *)((char *)(blk) + META_SIZE))

/* List of blocks obtained with brk(); they are contiguous in memory. */
static struct block_meta *head;
static struct block_meta *tail;

/* ---------- helpers ---------- */

static void *request_heap(size_t bytes)
{
	void *p = sbrk((intptr_t)bytes);

	DIE(p == (void *)-1, "sbrk");
	return p;
}

/* Merge every run of adjacent free blocks into a single block. */
static void coalesce_all(void)
{
	struct block_meta *cur = head;

	while (cur && cur->next) {
		struct block_meta *nxt = cur->next;

		if (cur->status == STATUS_FREE && nxt->status == STATUS_FREE) {
			cur->size += META_SIZE + nxt->size;
			cur->next = nxt->next;
			if (nxt->next)
				nxt->next->prev = cur;
			else
				tail = cur;
		} else {
			cur = nxt;
		}
	}
}

/* Merge `blk` with its next block (assumed free). */
static void merge_next(struct block_meta *blk)
{
	struct block_meta *nxt = blk->next;

	blk->size += META_SIZE + nxt->size;
	blk->next = nxt->next;
	if (nxt->next)
		nxt->next->prev = blk;
	else
		tail = blk;
}

/* Truncate `blk` to `size` bytes if the remainder can hold another block. */
static void split_block(struct block_meta *blk, size_t size)
{
	if (blk->size < size + META_SIZE + ALIGNMENT)
		return;

	struct block_meta *rest =
		(struct block_meta *)((char *)blk + META_SIZE + size);

	rest->size = blk->size - size - META_SIZE;
	rest->status = STATUS_FREE;
	rest->prev = blk;
	rest->next = blk->next;
	if (blk->next)
		blk->next->prev = rest;
	else
		tail = rest;
	blk->next = rest;
	blk->size = size;
}

static struct block_meta *find_best(size_t size)
{
	struct block_meta *best = NULL;

	for (struct block_meta *cur = head; cur; cur = cur->next) {
		if (cur->status != STATUS_FREE || cur->size < size)
			continue;
		if (!best || cur->size < best->size)
			best = cur;
	}
	return best;
}

static void *map_block(size_t size)
{
	size_t total = META_SIZE + size;
	struct block_meta *blk = mmap(NULL, total, PROT_READ | PROT_WRITE,
				      MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	DIE(blk == MAP_FAILED, "mmap");
	blk->size = size;
	blk->status = STATUS_MAPPED;
	blk->prev = NULL;
	blk->next = NULL;
	return PAYLOAD_OF(blk);
}

static void *heap_block(size_t size)
{
	struct block_meta *blk;

	/* Heap preallocation: happens only once. */
	if (!head) {
		blk = request_heap(MMAP_THRESHOLD);
		blk->size = MMAP_THRESHOLD - META_SIZE;
		blk->status = STATUS_FREE;
		blk->prev = NULL;
		blk->next = NULL;
		head = tail = blk;
	} else {
		coalesce_all();
	}

	blk = find_best(size);
	if (blk) {
		split_block(blk, size);
		blk->status = STATUS_ALLOC;
		return PAYLOAD_OF(blk);
	}

	/* No fit: expand the last block if it is free... */
	if (tail->status == STATUS_FREE) {
		request_heap(size - tail->size);
		tail->size = size;
		tail->status = STATUS_ALLOC;
		return PAYLOAD_OF(tail);
	}

	/* ...otherwise append a brand new block. */
	blk = request_heap(META_SIZE + size);
	blk->size = size;
	blk->status = STATUS_ALLOC;
	blk->prev = tail;
	blk->next = NULL;
	tail->next = blk;
	tail = blk;
	return PAYLOAD_OF(blk);
}

static void *do_alloc(size_t size, size_t threshold)
{
	size = ALIGN(size);
	if (META_SIZE + size >= threshold)
		return map_block(size);
	return heap_block(size);
}

/* ---------- public API ---------- */

void *os_malloc(size_t size)
{
	if (size == 0)
		return NULL;
	return do_alloc(size, MMAP_THRESHOLD);
}

void os_free(void *ptr)
{
	if (!ptr)
		return;

	struct block_meta *blk = BLOCK_OF(ptr);

	if (blk->status == STATUS_MAPPED) {
		int rc = munmap(blk, META_SIZE + blk->size);

		DIE(rc == -1, "munmap");
	} else if (blk->status == STATUS_ALLOC) {
		blk->status = STATUS_FREE;
	}
}

void *os_calloc(size_t nmemb, size_t size)
{
	if (nmemb == 0 || size == 0)
		return NULL;
	if (size > SIZE_MAX / nmemb)
		return NULL;

	size_t total = nmemb * size;
	long page = sysconf(_SC_PAGESIZE);

	DIE(page <= 0, "sysconf");

	void *ptr = do_alloc(total, (size_t)page);

	memset(ptr, 0, total);
	return ptr;
}

void *os_realloc(void *ptr, size_t size)
{
	if (!ptr)
		return os_malloc(size);
	if (size == 0) {
		os_free(ptr);
		return NULL;
	}

	struct block_meta *blk = BLOCK_OF(ptr);

	if (blk->status == STATUS_FREE)
		return NULL;

	size = ALIGN(size);

	/* Mapped blocks, or heap blocks that must move to mmap: relocate. */
	if (blk->status == STATUS_MAPPED || META_SIZE + size >= MMAP_THRESHOLD) {
		void *np = os_malloc(size);

		memcpy(np, ptr, blk->size < size ? blk->size : size);
		os_free(ptr);
		return np;
	}

	/* Shrink (or same size). */
	if (size <= blk->size) {
		split_block(blk, size);
		return ptr;
	}

	/* Try to expand in place, coalescing one free neighbour at a time. */
	while (blk->size < size && blk->next && blk->next->status == STATUS_FREE)
		merge_next(blk);

	if (blk->size >= size) {
		split_block(blk, size);
		return ptr;
	}

	/* Last block on the heap: grow the heap. */
	if (blk == tail) {
		request_heap(size - blk->size);
		blk->size = size;
		return ptr;
	}

	/* Otherwise move. */
	void *np = os_malloc(size);

	memcpy(np, ptr, blk->size);
	os_free(ptr);
	return np;
}
