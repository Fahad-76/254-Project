#include "config.h"
#include "riscv.h"
#include <assert.h>
#include <getopt.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include "cache.h"
#include "pipeline.h"

/* WARNING: DO NOT CHANGE THIS FILE.
 YOU PROBABLY DON'T EVEN NEED TO LOOK AT IT... */

// Pointer to simulator memory
Byte *memory;
#define MAX_SIZE 50

void execute_emu(regfile_t *regfile, int prompt, int print) {
  /* fetch an instruction */
  uint32_t instruction_bits = load(memory, regfile->PC, LENGTH_WORD);

  /* interactive-mode prompt */
  if (prompt) {
    if (prompt == 1) {
      printf("emulator paused,enter to continue...");
      while (getchar() != '\n')
        ;
    }

    printf("%08x: ", regfile->PC);
    decode_instruction(instruction_bits);
  }

  execute_instruction(instruction_bits, regfile, memory);

  // enforce $0 being hard-wired to 0
  regfile->R[0] = 0;

  // print trace
  if (print) {
    int i, j;

    for (i = 0; i < 8; i++) {
      for (j = 0; j < 4; j++) {
        printf("r%2d=%08x ", i * 4 + j, regfile->R[i * 4 + j]);
      }

      puts("");
    }

    printf("\n");
  }
}

int load_program(uint8_t *mem, size_t memsize, int startaddr,
                 const char *filename, int disasm) {
  FILE *file = fopen(filename, "r");
  char line[MAX_SIZE];
  int instruction, offset = 0;
  int programsize = 0;
  while (fgets(line, MAX_SIZE, file) != NULL) {
    instruction = (int32_t)strtol(line, NULL, 16);
    // printf("[load_program]: Instruction: %x\n", instruction);
    programsize++;
    // printf("%x, %d, %d\n", instruction, startaddr, offset);
    mem[startaddr + offset] = instruction & 0xFF;
    mem[startaddr + offset + 1] = (instruction >> 8) & 0xFF;
    mem[startaddr + offset + 2] = (instruction >> 16) & 0xFF;
    mem[startaddr + offset + 3] = (instruction >> 24) & 0xFF;

    if (disasm) {
      printf("%08x: ", startaddr + offset);
      decode_instruction((uint32_t)instruction);
    }

    offset += 4;
  }
  return programsize;
}

int main(int argc, char **argv) {
  /* options */
  int opt_disasm = 0,
      opt_regdump = 0,
      opt_interactive = 0,
      opt_exit = 0,
      opt_mulator = 0,
      opt_sim = 0,
      opt_init_reg = 0,
      opt_cache = 0,
      opt_forwarding = 0,
      opt_printmem = 0;

  uint32_t print_mem_startaddr = 0, print_mem_stopaddr = 0;


  /* the architectural state of the CPU */
  regfile_t regfile;

  /* parse the command-line args */
  int c;
  while ((c = getopt(argc, argv, "dvritesmpcf")) != -1) {
    switch (c) {
    case 'd':
      opt_disasm = 1; break;
    case 'v':
      opt_init_reg = 1; break;
    case 'r':
      opt_regdump = 1; break;
    case 'i':
      opt_interactive = 1; break;
    case 't':
      opt_interactive = 2; break;
    case 'e':
      opt_exit = 1; break;
    case 's':
      opt_sim = 1; break;
    case 'm':
      opt_mulator = 1; break;
    case 'c':
      opt_cache = 1; break;
    case 'f':
      opt_forwarding = 1; break;
    case 'p':
      opt_printmem = 1;
      if (optind < argc - 1) { // Ensure there are two more arguments
          print_mem_startaddr = (uint32_t)strtoul(argv[optind],NULL,16);
          print_mem_stopaddr  = (uint32_t)strtoul(argv[optind+1],NULL,16);
      } else {
          printf("Option -p requires two arguments\n");
      }
      optind+=2; // Skip the next argument as it's part of -p
      break;
    default:
      fprintf(stderr, "Bad option %c\n", c);
      return -1;
    }
  }

  /* make sure we got an executable filename on the command line */
  if (argc <= optind) {
    fprintf(stderr, "Give me an executable file to run!\n");
    return -1;
  }
  
  Cache cache;
  cacheSetUp(&cache, "L1");
  /* load the executable into memory */
  assert(memory == NULL);
  memory = calloc(MEMORY_SPACE, sizeof(uint8_t)); // allocate zeroed memory
  assert(memory != NULL);
  int prog_numins = 0;
  /* set the PC to 0x1000 */
  regfile.PC = 0x1000;
  prog_numins = load_program(memory, MEMORY_SPACE, regfile.PC, argv[optind],
                             opt_disasm);
  /* if we're just disassembling, exit here */
  if (opt_disasm) {
    return 0;
  }

  /* initialize the CPU */
  /* zero out all registers */
  int i;
  for (i = 1; i < 32; i++) {
    if (!opt_init_reg)
      regfile.R[i] = 0;
    else
      regfile.R[i] = 4;
  }
  regfile.R[0] = 0;       // R[0] is always 0

  /* Set the global pointer to 0x3000. We arbitrarily call this the middle of
   * the static data segment */
  regfile.R[3] = 0x3000;

  /* Set the stack pointer near the top of the memory array */
  regfile.R[2] = 0xEFFFF;

  int simins = 0;

  pipeline_regs_t pipeline_regs = {0};
  pipeline_wires_t pipeline_wires = {0};
  total_cycle_counter = 0;
  mem_access_counter = 0;

  bootstrap(&pipeline_wires, &pipeline_regs, &regfile);

  // EMULATOR
  if(opt_mulator)
  {
    if (opt_exit) {
      /* simulate forever! */
      while (1) {
        execute_emu(&regfile, opt_interactive, opt_regdump);
      }
    } else {
      /* Either simulate for program instructions */
      while (simins < prog_numins) {
        execute_emu(&regfile, opt_interactive, opt_regdump);
        simins++;
      }
    }
  }

  // CYCLE ACCURATE SIMULATOR
  if(opt_sim)
  {
    if(opt_cache) sim_config.cache_en = true;
    if(opt_forwarding) sim_config.fwd_en = true;
    bool ecall_exit = false;
    if (opt_exit) {
      /* simulate forever! */
      while (1) {
        cycle_pipeline(&regfile, memory, &cache, &pipeline_regs, &pipeline_wires, &ecall_exit);
        if(ecall_exit) break;
      }
    } else {
      /* Either simulate for program instructions */
      while (simins < prog_numins) {
        cycle_pipeline(&regfile, memory, &cache, &pipeline_regs, &pipeline_wires, &ecall_exit);
        simins++;
      }
    }
    printf("\n========\n[MAIN]: Flushing pipeline\n========\n");
    simins = 0;
    prog_numins = load_program(memory, MEMORY_SPACE, pipeline_wires.pc_src0, "./code/input/FLUSH.input",
                            opt_disasm);
    while (simins < prog_numins) {
      cycle_pipeline(&regfile, memory, &cache, &pipeline_regs, &pipeline_wires, &ecall_exit);
      simins++;
    }

    #ifdef PRINT_STATS
    printf("#Cycles            = %5ld\n", total_cycle_counter);
    printf("#Forwards (EX-EX)  = %5ld\n", fwd_exex_counter);
    printf("#Forwards (EX-MEM) = %5ld\n", fwd_exmem_counter);
    printf("#Branches taken    = %5ld\n", branch_counter);
    printf("#Stalls            = %5ld\n", stall_counter);
    #endif
    #ifdef PRINT_CACHE_STATS
      #if defined(CACHE_ENABLE)
      printf("#MEM   stalls      = %5ld\n", ((miss_count*MEM_LATENCY) + ((hit_count+miss_count) * (CACHE_HIT_LATENCY-1))));
      #else
      printf("#MEM   stalls      = %5ld\n", (mem_access_counter*(MEM_LATENCY-1)));
      #endif
      printf("#Cache accesses    = %5ld\n", hit_count+miss_count);
      printf("#Cache hits        = %5ld\n", hit_count);
      printf("#Cache misses      = %5ld\n", miss_count);
    #endif

  }

  // print mem
  if(opt_printmem)
  {
    printf("Dumping memory from 0x%04x to 0x%04x\n",
                          print_mem_startaddr,
                          print_mem_stopaddr);
    for (uint32_t i = 0;
                  i < (print_mem_stopaddr-print_mem_startaddr);
                  i+=16)                      // increment 16 bytes at once
    {
      for (uint32_t j = 0; j < 16; j+=4)     // of 4 Words each = 16 bytes
      {
        uint32_t index = (print_mem_startaddr) + i + j;
        printf("M:0x%04x=%08x ", index, memory[index]);
      }
      printf("\n");
    }
    printf("\n");
  }

  // Deallocate the cache after all operations
  deallocate(&cache);
  return 0;
}
