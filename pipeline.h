#ifndef __PIPELINE_H__
#define __PIPELINE_H__

#include "config.h"
#include "types.h"
#include "cache.h"
#include <stdbool.h>
#define DEBUG_CYCLE 
#define DEBUG_REG_TRACE  

///////////////////////////////////////////////////////////////////////////////
/// Functionality
///////////////////////////////////////////////////////////////////////////////

extern simulator_config_t sim_config;
extern uint64_t miss_count; // counter for cache misses
extern uint64_t hit_count;  // counter for cache hits 
extern uint64_t total_cycle_counter; // global counter incremented every cycle
extern uint64_t stall_counter; // counts pipeline bubbles/stalls
extern uint64_t branch_counter; // # of branch instructions executed
extern uint64_t fwd_exex_counter; // measures data hazard mitigations
extern uint64_t fwd_exmem_counter; // tracks how often forwarding resolves

///////////////////////////////////////////////////////////////////////////////
/// RISC-V Pipeline Register Types
///////////////////////////////////////////////////////////////////////////////

typedef struct {
    uint32_t instr;        // Raw 32-bit instruction
    uint32_t instr_addr;   // Address of this instruction
    //uint32_t next_pc;      // PC + 4 (next sequential PC)
    bool valid;
} ifid_reg_t;

extern uint64_t mem_access_counter;

typedef struct {
    uint32_t instr;        // Raw instruction bits
    uint32_t instr_addr;   // PC of this instruction

    // Register values and IDs
    uint32_t rs1_val;
    uint32_t rs2_val;
    uint32_t imm;
    uint8_t  rs1;
    uint8_t  rs2;
    uint8_t  rd;

    // Control signals
    uint8_t  alu_op;
    bool     mem_read;
    bool     mem_write;
    bool     reg_write;
    bool     mem_to_reg;

    uint8_t funct3;
    uint8_t funct7;

    bool branch;
    bool valid;
} idex_reg_t;

typedef struct {
    uint32_t instr;
    uint32_t instr_addr;
    uint32_t alu_result;
    uint32_t rs1_val;    // <-- ADD THIS
    uint32_t rs2_val;
    uint8_t  rd;
    uint8_t  funct3;

    // Control signals
    bool mem_read;
    bool mem_write;
    bool reg_write;
    bool mem_to_reg;
    bool branch;
    bool valid;
} exmem_reg_t;


typedef struct {
    uint32_t instr;        // Raw instruction bits
    uint32_t instr_addr;
    uint32_t alu_result;   // Result from ALU
    uint32_t mem_data;     // Data loaded from memory (if load)
    uint8_t  rd;

    bool     reg_write;
    bool     mem_to_reg;
    bool     valid;
} memwb_reg_t;

///////////////////////////////////////////////////////////////////////////////
/// Register types with input and output variants for simulator
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
  ifid_reg_t inp;
  ifid_reg_t out;
}ifid_reg_pair_t;

typedef struct
{
  idex_reg_t inp;
  idex_reg_t out;
}idex_reg_pair_t;

typedef struct
{
  exmem_reg_t inp;
  exmem_reg_t out;
}exmem_reg_pair_t;

typedef struct
{
  memwb_reg_t inp;
  memwb_reg_t out;
}memwb_reg_pair_t;


///////////////////////////////////////////////////////////////////////////////
/// Functional pipeline requirements
///////////////////////////////////////////////////////////////////////////////

typedef struct
{
  ifid_reg_pair_t  ifid_preg;
  idex_reg_pair_t  idex_preg;
  exmem_reg_pair_t exmem_preg;
  memwb_reg_pair_t memwb_preg;
}pipeline_regs_t;

typedef struct
{
  bool      pcsrc;
  uint32_t  pc_src0;
  uint32_t  pc_src1;
  uint8_t   forward_rs1;
  uint8_t   forward_rs2;
  uint32_t  memwb_wdata;
  uint32_t  exmem_alu_result;
  bool      reg_write;
  bool      flush;
  uint32_t  write_data;
  uint32_t  read_address;
}pipeline_wires_t;

///////////////////////////////////////////////////////////////////////////////
/// Helper Function
///////////////////////////////////////////////////////////////////////////////
void update_forwarding_value(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p);

///////////////////////////////////////////////////////////////////////////////
/// Function definitions for different stages
///////////////////////////////////////////////////////////////////////////////

/**
 * output : ifid_reg_t
 **/ 
ifid_reg_t stage_fetch(pipeline_wires_t* pwires_p, regfile_t* regfile_p, Byte* memory_p);

/**
 * output : idex_reg_t
 **/ 
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p);

/**
 * output : exmem_reg_t
 **/ 
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t* pwires_p);

/**
 * output : memwb_reg_t
 **/ 
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t* pwires_p, Byte* memory, Cache* cache_p , regfile_t* regfile_p);

/**
 * output : write_data
 **/ 
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t* pwires_p, regfile_t* regfile_p);

void cycle_pipeline(regfile_t* regfile_p, Byte* memory_p, Cache* cache_p, pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, bool* ecall_exit);

void bootstrap(pipeline_wires_t* pwires_p, pipeline_regs_t* pregs_p, regfile_t* regfile_p);

#endif  // __PIPELINE_H__
