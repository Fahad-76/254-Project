#ifndef __STAGE_HELPERS_H__
#define __STAGE_HELPERS_H__

#include <stdio.h>
#include "utils.h"
#include "pipeline.h"

/// EXECUTE STAGE HELPERS ///

/**
 * input  : idex_reg_t
 * output : uint32_t alu_control signal
 **/
uint32_t gen_alu_control(idex_reg_t idex_reg)
{
    Instruction instr = parse_instruction(idex_reg.instr);

    switch (instr.opcode) {
        case 0x33: // R-type
            switch (instr.rtype.funct3) {
                case 0x0:
                    if (instr.rtype.funct7 == 0x20) return 0x1; // SUB
                    else if (instr.rtype.funct7 == 0x01) return 0x5; // MUL
                    else return 0x0; // ADD
                case 0x1: return 0x7; // SLL
                case 0x2: return 0x8; // SLT
                case 0x3: return 0x9; // SLTU
                case 0x4: return 0x4; // XOR
                case 0x5: return (instr.rtype.funct7 == 0x20) ? 0xB : 0xA; // SRA/SRL
                case 0x6: return 0x3; // OR
                case 0x7: return 0x2; // AND
                default:  return 0x0; 
            }

        case 0x13: // I-type arithmetic
            switch (instr.itype.funct3) {
                case 0x0: return 0x0; // ADDI
                case 0x2: return 0x8; // SLTI
                case 0x3: return 0x9; // SLTIU
                case 0x4: return 0x4; // XORI
                case 0x6: return 0x3; // ORI
                case 0x7: return 0x2; // ANDI
                case 0x1: return 0x7; // SLLI
                case 0x5: // SRLI or SRAI
                    return ((instr.itype.imm & 0x400) != 0) ? 0xB : 0xA;
                default: return 0x0;
            }

        case 0x03: // Load 
        case 0x23: // Store 
        case 0x6F: // JAL 
            return 0x0;

        case 0x63: // Branch 
            return 0x1; 

        case 0x37: // LUI 
            return 0x6;

        default:
            return 0x0;
    }
}


/**
 * input  : alu_inp1, alu_inp2, alu_control
 * output : uint32_t alu_result
 **/
uint32_t execute_alu(int32_t alu_inp1, int32_t alu_inp2, uint32_t alu_control)
{
    uint32_t result;

    switch (alu_control) {
        case 0x0: result = alu_inp1 + alu_inp2; break; // ADD
        case 0x1: result = alu_inp1 - alu_inp2; break; // SUB
        case 0x2: result = alu_inp1 & alu_inp2; break; // AND
        case 0x3: result = alu_inp1 | alu_inp2; break; // OR
        case 0x4: result = alu_inp1 ^ alu_inp2; break; // XOR
        case 0x5: result = alu_inp1 * alu_inp2; break; // MUL
        case 0x6: result = alu_inp2; break;            // LUI (pass imm)
        case 0x7: result = alu_inp1 << (alu_inp2 & 0x1F); break; // SLL
        case 0x8: result = (alu_inp1 < alu_inp2) ? 1 : 0; break; // SLT
        case 0x9: result = ((uint32_t)alu_inp1 < (uint32_t)alu_inp2) ? 1 : 0; break; // SLTU
        case 0xA: result = (uint32_t)alu_inp1 >> (alu_inp2 & 0x1F); break; // SRL
        case 0xB: result = ((int32_t)alu_inp1) >> (alu_inp2 & 0x1F); break; // SRA
        default:  result = 0xBADCAFFE; break;
    }
    return result;
}

/// DECODE STAGE HELPERS ///

/**
 * input  : Instruction
 * output : idex_reg_t
 **/
uint32_t gen_imm(Instruction instruction)
{
    int imm_val = 0;
    switch (instruction.opcode) {
        case 0x63: // B-type
            imm_val = get_branch_offset(instruction);
            break;

        case 0x23: // S-type
            imm_val = get_store_offset(instruction);
            break;

        case 0x03: // I-type load
            imm_val = sign_extend_number(instruction.itype.imm, 12);
            break;

        case 0x73: // ECALL/CSR
            imm_val = sign_extend_number(instruction.itype.imm, 12);
            break;

        case 0x13: // I-type arithmetic
        if (instruction.itype.funct3 == 0x1 || instruction.itype.funct3 == 0x5) {
            imm_val = instruction.itype.imm & 0x1F;  // shifts
        } else if (instruction.itype.funct3 == 0x6 || instruction.itype.funct3 == 0x4) {
            imm_val = instruction.itype.imm & 0xFFF; // zero-extend ORI/XORI
       } else {
            imm_val = sign_extend_number(instruction.itype.imm, 12);
    }
       break;


        case 0x6F: // JAL
            imm_val = get_jump_offset(instruction);
            break;

        case 0x37: // LUI
            imm_val = instruction.utype.imm << 12;
            break;

        default: // R-type or undefined
        imm_val = 0;
            break;
    }
   // printf("DEBUG: gen_imm: opcode = 0x%x funt3 = 0x%x raw_imm=0x%x sign_extended=%d\n" , instruction.opcode , instruction.itype.funct3 , imm_val , imm_val);
    return imm_val;
}


/**
 * generates all the control logic that flows around in the pipeline
 * input  : Instruction
 * output : idex_reg_t
 **/
idex_reg_t gen_control(Instruction instruction)
{
    idex_reg_t idex_reg = {0};

    switch (instruction.opcode) {
        case 0x33:  // R-type (ADD, SUB, etc.)
            idex_reg.reg_write = true;
            idex_reg.alu_op = 0;   // 0 = R-type (use rs2)
            break;

        case 0x13:  // I-type (ADDI, ANDI, etc.)
            idex_reg.reg_write = true;
            idex_reg.alu_op = 1;   // 1 = I-type (use imm)
            break;

        case 0x03:  // Load
            idex_reg.reg_write = true;
            idex_reg.mem_read = true;
            idex_reg.mem_to_reg = true;
            idex_reg.alu_op = 1;   // use imm
            break;

        case 0x23:  // Store
            idex_reg.mem_write = true;
            idex_reg.alu_op = 1;   // use imm
            break;

        case 0x63:  // Branch
            idex_reg.alu_op = 1;   // use imm (offset)
            break;

        case 0x37:  // LUI
            idex_reg.reg_write = true;
            idex_reg.alu_op = 1;   // use imm
            break;

        case 0x17:  // AUIPC
            idex_reg.reg_write = true;
            idex_reg.alu_op = 1;   // use imm
            break;

        case 0x6F:  // JAL
            idex_reg.reg_write = true;
            idex_reg.alu_op = 1;   // use imm
            break;

        default:
            idex_reg.alu_op = 1;   // Default to imm
            break;
    }

    return idex_reg;
}

/// MEMORY STAGE HELPERS ///

/**
 * evaluates whether a branch must be taken
 * input  : <open to implementation>
 * output : bool
 **/
bool gen_branch(Instruction instr, uint32_t rs1_val, uint32_t rs2_val)
{
  bool branch_taken = false;

  if(instr.opcode == 0x63){  // Branch
    switch(instr.sbtype.funct3){
      case 0x0: branch_taken = (rs1_val == rs2_val); break; // BEQ
      case 0x1: branch_taken = (rs1_val != rs2_val); break; // BNE
      case 0x4: branch_taken = ((int32_t)rs1_val < (int32_t)rs2_val); break; // BLT
      case 0x5: branch_taken = ((int32_t)rs1_val >= (int32_t)rs2_val); break; // BGE
      case 0x6: branch_taken = (rs1_val < rs2_val); break;  // BLTU
      case 0x7: branch_taken = (rs1_val >= rs2_val); break; // BGEU
      default:  branch_taken = false; break;
    }
  }
  return branch_taken;
}

/// PIPELINE FEATURES ///

/**
 * Task   : Sets the pipeline wires for the forwarding unit's control signals
 *           based on the pipeline register values.
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
*/
void gen_forward(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p)
{
  //did
  pwires_p->forward_rs1 = 0;
  pwires_p->forward_rs2 = 0;

  uint8_t id_rs1 = pregs_p->idex_preg.out.rs1;
  uint8_t id_rs2 = pregs_p->idex_preg.out.rs2;

  if(pregs_p->exmem_preg.out.reg_write && pregs_p->exmem_preg.out.rd != 0){
    if(pregs_p->exmem_preg.out.rd == id_rs1){
      pwires_p->forward_rs1 =2;
    }
    if(pregs_p->exmem_preg.out.rd == id_rs2){
      pwires_p->forward_rs2 = 2;
    }
  }
  
   if(pregs_p->memwb_preg.out.reg_write && pregs_p->memwb_preg.out.rd != 0){
    if(pregs_p->memwb_preg.out.rd == id_rs1 && pwires_p->forward_rs1 == 0){
      pwires_p->forward_rs1 =1;
    }
    if(pregs_p->memwb_preg.out.rd == id_rs2 && pwires_p->forward_rs2 ==0){
      pwires_p->forward_rs2 = 1;
    }
  }

}

/**
 * Task   : Sets the pipeline wires for the hazard unit's control signals
 *           based on the pipeline register values.
 * input  : pipeline_regs_t*, pipeline_wires_t*
 * output : None
*/
void detect_hazard(pipeline_regs_t* pregs_p, pipeline_wires_t* pwires_p, regfile_t* regfile_p)
{
  /**
   * YOUR CODE HERE
   */
}


///////////////////////////////////////////////////////////////////////////////


/// RESERVED FOR PRINTING REGISTER TRACE AFTER EACH CLOCK CYCLE ///
void print_register_trace(regfile_t* regfile_p)
{
  // print
  for (uint8_t i = 0; i < 8; i++)       // 8 columns
  {
    for (uint8_t j = 0; j < 4; j++)     // of 4 registers each
    {
      printf("r%2d=%08x ", i * 4 + j, regfile_p->R[i * 4 + j]);
    }
    printf("\n");
  }
  printf("\n");
}

#endif // __STAGE_HELPERS_H__
