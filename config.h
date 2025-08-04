#ifndef __CONFIG_H__
#define __CONFIG_H__

// For each test, uncomment all its macros, and disable all other macros.

// required for MS1 (test_simulator_ms1.sh)
// #define DEBUG_REG_TRACE	// prints the register trace
// enable `DEBUG_CYCLE` this after completing the code in each stage
// #define DEBUG_CYCLE
//#define MEM_LATENCY 0		// before ms3, we ignore memory access latency

// required for MS2 (test_simulator_ms2.sh)
// #define DEBUG_REG_TRACE
// #define DEBUG_CYCLE
// #define PRINT_STATS		// prints overall stats
// #define MEM_LATENCY 0

// required for MS2: vec_xprod.input (test_simulator_ms2_extended.sh)
//#define PRINT_STATS
//#define MEM_LATENCY 0

// required for MS3: (test_simulator_ms3.sh)
#define DEBUG_REG_TRACE
#define DEBUG_CYCLE
#define PRINT_STATS
#define MEM_LATENCY 100
#define CACHE_ENABLE 		// enable cache simulation
#define PRINT_CACHE_TRACES      // prints cache trace for each memory access 
#define PRINT_CACHE_STATS	// prints the cache stats at the end of program

#endif // __CONFIG_H__
