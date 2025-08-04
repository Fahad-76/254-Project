#include "cache.h"
#include "dogfault.h"
#include <assert.h>
#include <ctype.h>
#include <getopt.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// DO NOT MODIFY THIS FILE. INVOKE AFTER EACH ACCESS FROM runTrace
void print_result(result r) {
  if (r.status == CACHE_EVICT)
    printf(" [status: miss eviction, victim_block: 0x%llx, insert_block: 0x%llx]",
           r.victim_block_addr, r.insert_block_addr);
  if (r.status == CACHE_HIT)
    printf(" [status: hit]");
  if (r.status == CACHE_MISS)
    printf(" [status: miss, insert_block: 0x%llx]", r.insert_block_addr);
}

/* This is the entry point to operate the cache for a given address in the trace file.
 * First, is increments the global lru_clock in the corresponding cache set for the address.
 * Second, it checks if the address is already in the cache using the "probe_cache" function.
 * If yes, it is a cache hit:
 *     1) call the "hit_cacheline" function to update the counters inside the hit cache 
 *        line, including its lru_clock and access_counter.
 *     2) record a hit status in the return "result" struct and update hit_count 
 * Otherwise, it is a cache miss:
 *     1) call the "insert_cacheline" function, trying to find an empty cache line in the
 *        cache set and insert the address into the empty line. 
 *     2) if the "insert_cacheline" function returns true, record a miss status and the
          inserted block address in the return "result" struct and update miss_count
 *     3) otherwise, if the "insert_cacheline" function returns false:
 *          a) call the "victim_cacheline" function to figure which victim cache line to 
 *             replace based on the cache replacement policy (LRU and LFU).
 *          b) call the "replace_cacheline" function to replace the victim cache line with
 *             the new cache line to insert.
 *          c) record an eviction status, the victim block address, and the inserted block
 *             address in the return "result" struct. Update miss_count and eviction_count.
 */
result operateCache(const unsigned long long address, Cache *cache) {
    result r;
    unsigned long long set_index = cache_set(address, cache);
    Set *set = &cache->sets[set_index];

    set->lru_clock++;

    if (probe_cache(address, cache)) {
        hit_cacheline(address, cache);
        r.status = CACHE_HIT;
        cache->hit_count++;
    } else {
        if (insert_cacheline(address, cache)) {
            r.status = CACHE_MISS;
            r.insert_block_addr = address_to_block(address, cache);
            cache->miss_count++;
        } else {
            unsigned long long victim_addr = victim_cacheline(address, cache);
            replace_cacheline(victim_addr, address, cache);
            r.status = CACHE_EVICT;
            r.victim_block_addr = victim_addr;
            r.insert_block_addr = address_to_block(address, cache);
            cache->miss_count++;
            cache->eviction_count++;
        }
    }

    return r;
}


// HELPER FUNCTIONS USEFUL FOR IMPLEMENTING THE CACHE
// Given an address, return the block (aligned) address,
// i.e., byte offset bits are cleared to 0
unsigned long long address_to_block(const unsigned long long address,
                                const Cache *cache) {
  // did
  unsigned long long block_size = 1ULL << cache->blockBits; 
  return address & ~(block_size - 1);;
}

// Return the cache tag of an address
unsigned long long cache_tag(const unsigned long long address,
                             const Cache *cache) {
  return address >> (cache->blockBits + cache->setBits);
}


// Return the cache set index of the address
unsigned long long cache_set(const unsigned long long address,
                             const Cache *cache) {
  unsigned long long set_mask = (1ULL << cache->setBits) - 1;
  return (address >> cache->blockBits) & set_mask;
}


// Check if the address is found in the cache. If so, return true. else return false.
bool probe_cache(const unsigned long long address, const Cache *cache) {
  // did
  unsigned long long tag = cache_tag(address , cache);
  unsigned long long set_index = cache_set(address,cache);
  Set *set = &cache->sets[set_index];

  for (int i=0; i < cache->linesPerSet ; i++ ){
  Line *line = &set->lines[i];
    if(line->valid && line->tag == tag){
      return true;
    }
  }
  return false;
}

// Access address in cache. Called only if probe is successful.
// Update the LRU (least recently used) or LFU (least frequently used) counters.
void hit_cacheline(const unsigned long long address, Cache *cache){
  //did

  unsigned long long tag = cache_tag(address,cache);
  unsigned long long set_index = cache_set(address,cache);
  Set *set = &cache->sets[set_index];

  for (int i =0; i < cache->linesPerSet ; i++){
    Line *line = &set->lines[i];
    if(line->valid && line->tag == tag){
      line->lru_clock = set->lru_clock;
      line->access_counter +=1;
      return;
    }
  }
 }

/* This function is only called if probe_cache returns false, i.e., the address is
 * not in the cache. In this function, it will try to find an empty (i.e., invalid)
 * cache line for the address to insert. 
 * If it found an empty one:
 *     1) it inserts the address into that cache line (marking it valid).
 *     2) it updates the cache line's lru_clock based on the global lru_clock 
 *        in the cache set and initiates the cache line's access_counter.
 *     3) it returns true.
 * Otherwise, it returns false.  
 */ 
bool insert_cacheline(const unsigned long long address, Cache *cache) {
  //did
   unsigned long long tag = cache_tag(address,cache);
   unsigned long long set_index = cache_set(address,cache);
   Set *set = &cache->sets[set_index];

   for (int i = 0; i < cache->linesPerSet ; i++){
    Line *line = &set->lines[i];
    if(!line->valid){
      line->tag = tag;
      line->valid = true;
      line->lru_clock = set->lru_clock;
      line->access_counter = 1;
      return true;
    }
   }
   return false;
}

// If there is no empty cacheline, this method figures out which cacheline to replace
// depending on the cache replacement policy (LRU and LFU). It returns the block address
// of the victim cacheline; note we no longer have access to the full address of the victim
unsigned long long victim_cacheline(const unsigned long long address, const Cache *cache) {
    unsigned long long set_index = cache_set(address, cache);
    Set *set = &cache->sets[set_index];

    int victim_index = 0;

    if (cache->lfu == 0) {
        // LRU policy: least recently used = smallest lru_clock
        unsigned long long min_lru = set->lines[0].lru_clock;
        for (int i = 1; i < cache->linesPerSet; i++) {
            if (set->lines[i].lru_clock < min_lru) {
                min_lru = set->lines[i].lru_clock;
                victim_index = i;
            }
        }
    } else {
        // LFU policy: smallest access_counter, break ties with lru_clock
        int min_access = set->lines[0].access_counter;
        unsigned long long min_lru = set->lines[0].lru_clock;
        for (int i = 1; i < cache->linesPerSet; i++) {
            int access = set->lines[i].access_counter;
            unsigned long long lru = set->lines[i].lru_clock;
            if (access < min_access || (access == min_access && lru < min_lru)) {
                min_access = access;
                min_lru = lru;
                victim_index = i;
            }
        }
    }

    Line *victim = &set->lines[victim_index];

    int offset_bits = cache->blockBits;
    int set_bits = cache->setBits;

    unsigned long long block_address =
        (victim->tag << (offset_bits + set_bits)) |
        (set_index << offset_bits);

    return block_address;
}


/* Replace the victim cacheline with the new address to insert. Note for the victim cachline,
 * we only have its block address. For the new address to be inserted, we have its full address.
 * Remember to update the new cache line's lru_clock based on the global lru_clock in the cache
 * set and initiate the cache line's access_counter.
 */
void replace_cacheline(const unsigned long long victim_block_addr,
		       const unsigned long long insert_addr, Cache *cache) {
  //did
  unsigned long long victim_tag = cache_tag(victim_block_addr,cache);
  unsigned long long set_index = cache_set(victim_block_addr,cache);
  unsigned long long replacetag = cache_tag(insert_addr,cache);
  
  Set *set = &cache->sets[set_index];
  int victim_line = 0;
  for (int i =0; i < cache->linesPerSet ; i++){
    Line *line = &set->lines[i];
    if(line->valid && line->tag == victim_tag){
     victim_line = i;
     break;
    }
  }
   Line *victim = &set->lines[victim_line];
   victim->tag = replacetag;
   victim->valid = true;
   victim->lru_clock = set->lru_clock;
   victim->access_counter = 1;

}


// allocate the memory space for the cache with the given cache parameters
// and initialize the cache sets and lines.
// Initialize the cache name to the given name 
void cacheSetUp(Cache *cache, char *name) {
  //did
  int setBits = 1 << cache->setBits;
  int linesPerSet = cache->linesPerSet;

  cache->sets = (Set *)malloc(sizeof(Set)*setBits);
  if (cache->sets == NULL) {
    fprintf(stderr, "Error: Failed to allocate memory for sets.\n");
    exit(1);
    }

   for(int i = 0; i < setBits ; i++){
    cache->sets[i].lines = malloc(sizeof(Line) * linesPerSet);
    if(cache->sets[i].lines==NULL){
      fprintf(stderr, "Error: Failed to allocate memory for sets.\n");
    exit(1);
    }
    for(int j=0; j < linesPerSet; j++){
      cache->sets[i].lines[j].valid = false;
      cache->sets[i].lines[j].tag = 0;
      cache->sets[i].lines[j].lru_clock = 0;
      cache->sets[i].lines[j].access_counter=0;
    }
    cache->sets[i].lru_clock = 0;
   } 

   cache->hit_count = 0;
   cache->miss_count = 0;
   cache->eviction_count = 0;

   cache->name = strdup(name);  // Duplicates string, caller's input not modified
    if (cache->name == NULL) {
        fprintf(stderr, "Error: Failed to allocate memory for cache name.\n");
        exit(1);
    }
}

// deallocate the memory space for the cache
void deallocate(Cache *cache) {
  /* YOUR CODE HERE */
  int setBits = 1 << cache->setBits;
    for (int i = 0; i < setBits; i++) {
    if (cache->sets[i].lines != NULL) {
     free(cache->sets[i].lines);
     cache->sets[i].lines = NULL;
        }
    }
    if (cache->sets != NULL) {
     free(cache->sets);
     cache->sets = NULL;
    }
    if (cache->name != NULL) {
     free(cache->name);
     cache->name = NULL;
    }
}

// print out summary stats for the cache
void printSummary(const Cache *cache) {
  printf("%s hits: %d, misses: %d, evictions: %d\n", cache->name, cache->hit_count,
         cache->miss_count, cache->eviction_count);
}
