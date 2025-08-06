#include <stdbool.h>
#include "cache.h"
#include "riscv.h"
#include "types.h"
#include "utils.h"
#include "pipeline.h"
#include "stage_helpers.h"

uint64_t total_cycle_counter = 0;
uint64_t miss_count = 0;
uint64_t hit_count = 0;
uint64_t stall_counter = 0;
uint64_t branch_counter = 0;
uint64_t fwd_exex_counter = 0;
uint64_t mem_access_counter = 0; 
uint64_t fwd_exmem_counter = 0;

simulator_config_t sim_config = {0};

///////////////////////////////////////////////////////////////////////////////

void bootstrap(pipeline_wires_t* pwires_p, pipeline_regs_t* pregs_p, regfile_t* regfile_p)
{
    pwires_p->pc_src0 = regfile_p->PC;
}

///////////////////////////
/// STAGE FUNCTIONALITY ///
///////////////////////////

/**
 * STAGE  : stage_fetch
 * output : ifid_reg_t
 **/
ifid_reg_t stage_fetch(pipeline_wires_t *pwires_p, regfile_t *regfile_p, Byte *memory_p) {
    ifid_reg_t ifid_reg = {0};
     if (pwires_p->pcsrc) {
        regfile_p->PC = pwires_p->pc_src1;  // Redirect PC to branch target
        pwires_p->pcsrc = false;            // Clear signal after use
    }

    // ----------------------------
    // Fetch instruction from memory
    // ----------------------------
    uint32_t instruction_bits;
    memcpy(&instruction_bits, memory_p + regfile_p->PC, 4); // Load 32-bit word
    ifid_reg.instr = instruction_bits;
    ifid_reg.instr_addr = regfile_p->PC;

    // ----------------------------
    // Set default next PC (PC + 4)
    // ----------------------------
    pwires_p->pc_src0 = regfile_p->PC + 4;

#ifdef DEBUG_CYCLE
    printf("[IF ]: Instruction [%08x]@[%08x]: ", instruction_bits, regfile_p->PC);
    decode_instruction(instruction_bits);
#endif

    return ifid_reg;
}


/**
 * STAGE  : stage_decode
 * output : idex_reg_t
 **/ 
idex_reg_t stage_decode(ifid_reg_t ifid_reg, pipeline_wires_t *pwires_p, regfile_t *regfile_p) {
    idex_reg_t idex_reg = {0};  // Initialize ID/EX pipeline register

    // Handle NOP (no instruction)
    if (ifid_reg.instr == 0) {
#ifdef DEBUG_CYCLE
        printf("[ID ]: Instruction [%08x]@[%08x]: ", ifid_reg.instr, ifid_reg.instr_addr);
        decode_instruction(ifid_reg.instr);
#endif
        return idex_reg;
    }

    // Pass instruction and PC
    idex_reg.instr      = ifid_reg.instr;
    idex_reg.instr_addr = ifid_reg.instr_addr;

    // Parse instruction
    Instruction decoded = parse_instruction(ifid_reg.instr);

    // Initialize rs/rd
    uint8_t rs1 = 0, rs2 = 0, rd = 0;

    switch (decoded.opcode) {
        case 0x33: // R-type
            rs1 = decoded.rtype.rs1;
            rs2 = decoded.rtype.rs2;
            rd  = decoded.rtype.rd;
            break;

        case 0x13: // I-type (ALU immediate)
        case 0x03: // I-type (load)
            rs1 = decoded.itype.rs1;
            rd  = decoded.itype.rd;
            break;

        case 0x23: // S-type (store)
            rs1 = decoded.stype.rs1;
            rs2 = decoded.stype.rs2;
            break;

        case 0x63: // SB-type (branch)
            rs1 = decoded.sbtype.rs1;
            rs2 = decoded.sbtype.rs2;
            break;

        case 0x37: // U-type (LUI)
        case 0x17: // U-type (AUIPC)
            rd = decoded.utype.rd;
            break;

        case 0x6F: // UJ-type (JAL)
            rd = decoded.ujtype.rd;
            break;

        default:
            break;
    }

    // Assign registers
    idex_reg.rs1 = rs1;
    idex_reg.rs2 = rs2;
    idex_reg.rd  = rd;

    // Fetch register values
    idex_reg.rs1_val = regfile_p->R[rs1];
    idex_reg.rs2_val = regfile_p->R[rs2];

    // Immediate generation (UJ, SB, U properly handled)
    idex_reg.imm = gen_imm(decoded);

    // Control signals
    idex_reg_t control_signals = gen_control(decoded);
    idex_reg.mem_read   = control_signals.mem_read;
    idex_reg.mem_write  = control_signals.mem_write;
    idex_reg.reg_write  = control_signals.reg_write;
    idex_reg.mem_to_reg = control_signals.mem_to_reg;
    idex_reg.alu_op     = control_signals.alu_op;

    // Extra fields for ALU/branch decisions
    idex_reg.funct3 = (ifid_reg.instr >> 12) & 0x07;
    idex_reg.funct7 = (ifid_reg.instr >> 25) & 0x7F;

    // Branch-specific flag (optional)
    if (decoded.opcode == 0x63) {
        idex_reg.branch = gen_branch(decoded, regfile_p->R[rs1], regfile_p->R[rs2]);

    }

#ifdef DEBUG_CYCLE
    printf("[ID ]: Instruction [%08x]@[%08x]: ", ifid_reg.instr, ifid_reg.instr_addr);
    decode_instruction(ifid_reg.instr);
#endif

    return idex_reg;
}
/**
 * STAGE  : stage_execute
 * output : exmem_reg_t
 **/
exmem_reg_t stage_execute(idex_reg_t idex_reg, pipeline_wires_t *pwires_p) {
    exmem_reg_t exmem_reg = (exmem_reg_t){0};

    // Handle bubble (NOP)
    if (idex_reg.instr == 0) {
#ifdef DEBUG_CYCLE
        printf("[EX ]: Instruction [%08x]@[%08x]: ", idex_reg.instr, idex_reg.instr_addr);
        decode_instruction(idex_reg.instr);
#endif
        return exmem_reg;
    }

    Instruction instr = parse_instruction(idex_reg.instr);

    // ---------------------------
    // Determine operands
    // ---------------------------
    uint32_t operand1 = idex_reg.rs1_val;
    uint32_t operand2;

    // R-type uses rs2, others use imm
    if (instr.opcode == 0x33) {
        operand2 = idex_reg.rs2_val;
    } else {
        operand2 = idex_reg.imm;
    }

    // ---------------------------
    // ALU operation
    // ---------------------------
    uint32_t alu_ctrl   = gen_alu_control(idex_reg);
    uint32_t alu_result = execute_alu(operand1, operand2, alu_ctrl);

    // ---------------------------
    // Handle special opcodes
    // ---------------------------
    if (instr.opcode == 0x6F) {          // JAL
        alu_result = idex_reg.instr_addr + 4;  // link address
    } else if (instr.opcode == 0x37) {   // LUI
        alu_result = idex_reg.imm;
    } else if (instr.opcode == 0x17) {   // AUIPC
        alu_result = idex_reg.instr_addr + idex_reg.imm;
    }

    // ---------------------------
    // Populate EX/MEM register
    // ---------------------------
    exmem_reg.instr       = idex_reg.instr;
    exmem_reg.instr_addr  = idex_reg.instr_addr;
    exmem_reg.alu_result  = alu_result;
    exmem_reg.rs1_val     = idex_reg.rs1_val;   // needed for branch
    exmem_reg.rs2_val     = idex_reg.rs2_val;
    exmem_reg.rd          = idex_reg.rd;
    exmem_reg.funct3      = idex_reg.funct3;

    // Control signals
    exmem_reg.mem_read    = idex_reg.mem_read;
    exmem_reg.mem_write   = idex_reg.mem_write;
    exmem_reg.reg_write   = idex_reg.reg_write;
    exmem_reg.mem_to_reg  = idex_reg.mem_to_reg;
    exmem_reg.branch      = idex_reg.branch;

    // Update pipeline wires (optional)
    pwires_p->read_address = exmem_reg.alu_result;

#ifdef DEBUG_CYCLE
    printf("[EX ]: Instruction [%08x]@[%08x]: ", idex_reg.instr, idex_reg.instr_addr);
    decode_instruction(idex_reg.instr);
#endif

    return exmem_reg;
}


 //STAGE  : stage_mem
memwb_reg_t stage_mem(exmem_reg_t exmem_reg, pipeline_wires_t *pwires_p, Byte *memory_p, Cache *cache_p) {
    memwb_reg_t memwb_reg = {0};

    // Bubble check
    if (exmem_reg.instr == 0) {
#ifdef DEBUG_CYCLE
        printf("[MEM]: Instruction [%08x]@[%08x]: ", exmem_reg.instr, exmem_reg.instr_addr);
        decode_instruction(exmem_reg.instr);
#endif
        return memwb_reg;
    }

    // Parse instruction ONCE here
    Instruction instr = parse_instruction(exmem_reg.instr);

    // --- Copy fields ---
    memwb_reg.instr      = exmem_reg.instr;
    memwb_reg.instr_addr = exmem_reg.instr_addr;
    memwb_reg.alu_result = exmem_reg.alu_result;
    memwb_reg.rd         = exmem_reg.rd;
    memwb_reg.mem_to_reg = exmem_reg.mem_to_reg;
    memwb_reg.reg_write  = exmem_reg.reg_write;

    // --- Load handling ---
    if (exmem_reg.mem_read) {
        switch (instr.itype.funct3) {
            case 0x0: { int8_t val;  memcpy(&val, memory_p + exmem_reg.alu_result, 1); memwb_reg.mem_data = (int32_t)val; break; }
            case 0x1: { int16_t val; memcpy(&val, memory_p + exmem_reg.alu_result, 2); memwb_reg.mem_data = (int32_t)val; break; }
            case 0x2: memcpy(&memwb_reg.mem_data, memory_p + exmem_reg.alu_result, 4); break;
            case 0x4: { uint8_t val;  memcpy(&val, memory_p + exmem_reg.alu_result, 1); memwb_reg.mem_data = val; break; }
            case 0x5: { uint16_t val; memcpy(&val, memory_p + exmem_reg.alu_result, 2); memwb_reg.mem_data = val; break; }

        }
    }

    // --- Store handling ---
    if (exmem_reg.mem_write) {
        switch (instr.stype.funct3) {
            case 0x0: memcpy(memory_p + exmem_reg.alu_result, &exmem_reg.rs2_val, 1); break;
            case 0x1: memcpy(memory_p + exmem_reg.alu_result, &exmem_reg.rs2_val, 2); break;
            case 0x2: memcpy(memory_p + exmem_reg.alu_result, &exmem_reg.rs2_val, 4); break;
        }
    }

    // --- Branch/JAL handling ---
    if (instr.opcode == 0x63) {
        if (gen_branch(instr, exmem_reg.rs1_val, exmem_reg.rs2_val)) {
            pwires_p->pcsrc = true;
            pwires_p->pc_src1 = exmem_reg.instr_addr + get_branch_offset(instr);
        }
    } else if (instr.opcode == 0x6F) {
        pwires_p->pcsrc = true;
        pwires_p->pc_src1 = exmem_reg.instr_addr + get_jump_offset(instr);
    }

#ifdef DEBUG_CYCLE
    printf("[MEM]: Instruction [%08x]@[%08x]: ", exmem_reg.instr, exmem_reg.instr_addr);
    decode_instruction(exmem_reg.instr);
#endif

    return memwb_reg;
}


/**
 * STAGE  : stage_writeback
 * output : nothing - The state of the register file may be changed
 **/
void stage_writeback(memwb_reg_t memwb_reg, pipeline_wires_t *pwires_p, regfile_t *regfile_p) {
    // Handle bubble (NOP)
    if (memwb_reg.instr == 0) {
#ifdef DEBUG_CYCLE
        printf("[WB ]: Instruction [%08x]@[%08x]: ", memwb_reg.instr, memwb_reg.instr_addr);
        decode_instruction(memwb_reg.instr);
#endif
        return;
    }

    // ----------------------------
    // Determine value to write back
    // ----------------------------
    uint32_t write_value = 0;
    if (memwb_reg.mem_to_reg) {
        // Value from memory (load instruction)
        write_value = memwb_reg.mem_data;
    } else {
        // Value from ALU
        write_value = memwb_reg.alu_result;
    }

    // ----------------------------
    // Write to register file
    // ----------------------------
    if (memwb_reg.reg_write && memwb_reg.rd != 0) {
        regfile_p->R[memwb_reg.rd] = write_value;
    }

    // ----------------------------
    // Update debug wires
    // ----------------------------
    pwires_p->reg_write = memwb_reg.reg_write;

#ifdef DEBUG_CYCLE
    printf("[WB ]: Instruction [%08x]@[%08x]: ", memwb_reg.instr, memwb_reg.instr_addr);
    decode_instruction(memwb_reg.instr);
#endif
}


///////////////////////////////////////////////////////////////////////////////
/**
 * Excite the pipeline with one clock cycle
 **/
void cycle_pipeline(regfile_t *regfile_p, Byte *memory_p, Cache *cache_p,
                    pipeline_regs_t *pregs_p, pipeline_wires_t *pwires_p, bool *ecall_exit) {
#ifdef DEBUG_CYCLE
    printf("v==============");
    printf("Cycle Counter = %5ld", total_cycle_counter);
    printf("==============v\n\n");
#endif

    // --- Stage progression ---
    pregs_p->ifid_preg.inp  = stage_fetch(pwires_p, regfile_p, memory_p);
    pregs_p->idex_preg.inp  = stage_decode(pregs_p->ifid_preg.out, pwires_p, regfile_p);
    pregs_p->exmem_preg.inp = stage_execute(pregs_p->idex_preg.out, pwires_p);
    pregs_p->memwb_preg.inp = stage_mem(pregs_p->exmem_preg.out, pwires_p, memory_p, cache_p);
                              stage_writeback(pregs_p->memwb_preg.out, pwires_p, regfile_p);

    // --- Handle PC update ---
    if (pwires_p->pcsrc) {
    #ifdef DEBUG_CYCLE
        printf("\n========\n[MAIN]: Flushing pipeline\n========\n");
    #endif

        regfile_p->PC = pwires_p->pc_src1;
        //pregs_p->ifid_preg.out = (ifid_reg_t){0}; // Flush only IF stage
        pregs_p->ifid_preg.out.instr = 0x00000013;  // addi x0,x0,0

        pwires_p->pcsrc = false;
} else {
    regfile_p->PC = pwires_p->pc_src0; // Sequential
}


    // --- Advance pipeline registers ---
    pregs_p->ifid_preg.out  = pregs_p->ifid_preg.inp;
    pregs_p->idex_preg.out  = pregs_p->idex_preg.inp;
    pregs_p->exmem_preg.out = pregs_p->exmem_preg.inp;
    pregs_p->memwb_preg.out = pregs_p->memwb_preg.inp;


  /////////////////// NO CHANGES BELOW THIS ARE REQUIRED //////////////////////

  // increment the cycle
  total_cycle_counter++;

  #ifdef DEBUG_REG_TRACE
  print_register_trace(regfile_p);
  #endif

  /**
   * check ecall condition
   * To do this, the value stored in R[10] (a0 or x10) should be 10.
   * Hence, the ecall condition is checked by the existence of following
   * two instructions in sequence:
   * 1. <instr>  x10, <val1>, <val2> 
   * 2. ecall
   * 
   * The first instruction must write the value 10 to x10.
   * The second instruction is the ecall (opcode: 0x73)
   * 
   * The condition checks whether the R[10] value is 10 when the
   * `memwb_reg.instr.opcode` == 0x73 (to propagate the ecall)
   * 
   * If more functionality on ecall needs to be added, it can be done
   * by adding more conditions on the value of R[10]
   */
  if( (pregs_p->memwb_preg.out.instr == 0x00000073) &&
      (regfile_p->R[10] == 10) )
  {
    *(ecall_exit) = true;
  }
}

