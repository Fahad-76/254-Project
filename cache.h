#ifndef CACHE_H
#define CACHE_H
#include "dogfault.h"
#include <assert.h>
#include <ctype.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "utils.h"
#include "config.h"
enum status_enum {
  CACHE_MISS = 0,
  CACHE_HIT = 1,
  CACHE_EVICT = 2
};

#define CACHE_HIT_LATENCY 2    // hit latency
#define CACHE_MISS_LATENCY MEM_LATENCY+CACHE_HIT_LATENCY  // miss latency
#define CACHE_OTHER_LATENCY MEM_LATENCY+CACHE_HIT_LATENCY // eviction latency
#define CACHE_SET_BITS 4 // number of sets (2^CACHE_SET_BITS)
#define CACHE_LINES_PER_SET 4 // Number of lines per set (associativity)
#define CACHE_BLOCK_BITS 6 // number of blocks (2^CACHE_BLOCK_BITS)
#define CACHE_DISPLAY_TRACE false
#define CACHE_LFU 1 // LRU

// Struct definitions
typedef struct {
    bool valid;
    unsigned long long tag;
    unsigned long long block_addr;
    int lru_clock;
    int access_counter;
} Line;

typedef struct {
    Line *lines;
    int lru_clock;
} Set;

typedef struct {
    Set *sets;
    int hit_count;
    int miss_count;
    int eviction_count;
    int lfu;
    bool displayTrace;
    int setBits;
    int linesPerSet;
    int blockBits;
    char *name;
} Cache;

typedef struct {
    int status;
    unsigned long long insert_block_addr;
    unsigned long long victim_block_addr;
} result;

// Function declarations
void cacheSetUp(Cache *cache, char *name);
void deallocate(Cache *cache);
result operateCache(const unsigned long long address, Cache *cache);
int processCacheOperation(unsigned long address, Cache *cache);
unsigned long long address_to_block(const unsigned long long address, const Cache *cache);
unsigned long long cache_tag(const unsigned long long address, const Cache *cache);
unsigned long long cache_set(const unsigned long long address, const Cache *cache);
bool probe_cache(const unsigned long long address, const Cache *cache);
void hit_cacheline(const unsigned long long address, Cache *cache);
bool insert_cacheline(const unsigned long long address, Cache *cache);
unsigned long long victim_cacheline(const unsigned long long address, const Cache *cache);
void replace_cacheline(const unsigned long long victim_block_addr, const unsigned long long insert_addr, Cache *cache);
#endif // CACHE_H
