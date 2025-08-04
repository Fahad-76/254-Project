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
        case 0x33: // rtype
            switch (instr.rtype.funct3) {
                case 0x0:
                    if (instr.rtype.funct7 == 0x20) return 0x1; //sub
                    else if (instr.rtype.funct7 == 0x1) return 0x5; // mul
                    else return 0x0; // add
                case 0x1: return 0x7; //sll
                case 0x2: return 0x8; //slt
                case 0x3: return 0x9; //sltu
                case 0x4: return 0x4; //xor
                case 0x5: return (instr.rtype.funct7 == 0x20)? 0xB : 0xA; //srl.sra 
                case 0x7: return 0x2;
                case 0x6: return 0x3;               
                default:  return 0x0; 
            }
            break;

        case 0x13: //itype
        case 0x03: // load
        case 0x23: // store
        case 0x6F: // jump
            return 0x0;

        case 0x63: // Branch 
            return 0x1;

        case 0x37: // 
            return 0x6;

        default:
            return 0x0;
    }
  //return alu_control;
}

/**
 * input  : alu_inp1, alu_inp2, alu_control
 * output : uint32_t alu_result
 **/
uint32_t execute_alu(uint32_t alu_inp1, uint32_t alu_inp2, uint32_t alu_control)
{
  uint32_t result;
  switch(alu_control){
    case 0x0: //add
      result = alu_inp1 + alu_inp2;
      break;
    
    case 0x1:
      result = alu_inp1 - alu_inp2;
      break;
    
    case 0x2:
       result = alu_inp1 & alu_inp2;
       break;

    case 0x3:
       result = alu_inp1 | alu_inp2;
       break;

    case 0x4:
       result = alu_inp1 ^ alu_inp2;
       break;
    
    case 0x5:
       result = alu_inp1 * alu_inp2;
       break;

    case 0x6:
       result = alu_inp2;
       break;
    
    case 0x7:
       result = alu_inp1 << (alu_inp2 & 0x1F);
       break;

    case 0x8:
       result = ((int32_t)alu_inp1 < (int32_t)alu_inp2) ? 1:0;
       break;

    case 0x9:
       result = (alu_inp1 < alu_inp2) ? 1 : 0;
       break;

    case 0xA:
       result = alu_inp1 >> (alu_inp2 & 0x1F);
       break;

    case 0xB:
       result = (int32_t)alu_inp1 >> (alu_inp2 & 0x1F);
       break;

    default:
      result = 0xBADCAFFE;
      break;
  };
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
  /**
   * YOUR CODE HERE
   */
  switch(instruction.opcode) {
        case 0x63: //B-type
            imm_val = get_branch_offset(instruction);
            break;
        //did

        case 0x23: //stype
         imm_val = get_store_offset(instruction);
         break;

        case 0x03:
        case 0x13:
        case 0x73:
          imm_val = sign_extend_number(instruction.itype.imm, 12);
          break;

        case 0x6F:
          imm_val = get_jump_offset(instruction);
          break;

        case 0x37:
          imm_val = instruction.utype.imm;
          break;

        default: // R and undefined opcode
            break;
    };
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
  switch(instruction.opcode) {
      case 0x33:  //R-type
        idex_reg.reg_write = true;
        idex_reg.mem_read = false;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = 0x33;
        break;

      case 0x13:  //itype
        idex_reg.reg_write = true;
        idex_reg.mem_read = false;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = 0x13;
        break;

      case 0x03: //load
        idex_reg.reg_write = true;
        idex_reg.mem_read = true;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = true;
        idex_reg.alu_op     = 0x03;
        break;

      case 0x23: //store
        idex_reg.reg_write = false;
        idex_reg.mem_read = false;
        idex_reg.mem_write = true;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = 0x23;
        break;

      case 0x63: //branch
        idex_reg.reg_write = false;
        idex_reg.mem_read = false;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = 0x63;
        break;

      case 0x37: //link
        idex_reg.reg_write = true;
        idex_reg.mem_read = false;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = 0x37;
        break;

      case 0x6F: //jump and link
        idex_reg.reg_write = true;
        idex_reg.mem_read = false;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = 0x6F;
        break;
      default:  // Remaining opcodes
        idex_reg.reg_write = false;
        idex_reg.mem_read = false;
        idex_reg.mem_write = false;
        idex_reg.mem_to_reg = false;
        idex_reg.alu_op     = instruction.opcode;
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
  //did
  bool branch_taken = false;

  if(instr.opcode == 0x63){  //branch
    switch(instr.sbtype.funct3){
      case 0x0: //beq
       branch_taken = (rs1_val == rs2_val);
       break;

      case 0x1://bne
       branch_taken = (rs1_val != rs2_val);
       break;

      case 0x4: //blt
       branch_taken = ((int32_t)rs1_val < (int32_t)rs2_val);
       break;

      case 0x5: //bge
       branch_taken = ((int32_t)rs1_val >= (int32_t)rs2_val);
       break;
      
      case 0x6: //bltu
       branch_taken = (rs1_val < rs2_val);
       break;

      case 0x7: //bgeu
       branch_taken = (rs1_val >= rs2_val);
       break;

      default:
        branch_taken = false;
        break;
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
  pwires_p->fwd_a = 0;
  pwires_p->fwd_b = 0;

  uint8_t id_rs1 = pregs_p->idex_preg.out.rs1;
  uint8_t id_rs2 = pregs_p->idex_preg.out.rs2;

  if(pregs_p->exmem_preg.out.reg_write && pregs_p->exmem_preg.out.rd != 0){
    if(pregs_p->exmem_preg.out.rd == id_rs1){
      pwires_p->fwd_a =2;
    }
    if(pregs_p->exmem_preg.out.rd == id_rs2){
      pwires_p->fwd_b = 2;
    }
  }
  
   if(pregs_p->memwb_preg.out.reg_write && pregs_p->memwb_preg.out.rd != 0){
    if(pregs_p->memwb_preg.out.rd == id_rs1 && pwires_p->fwd_a == 0){
      pwires_p->fwd_a =1;
    }
    if(pregs_p->memwb_preg.out.rd == id_rs2 && pwires_p->fwd_b ==0){
      pwires_p->fwd_b = 1;
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
