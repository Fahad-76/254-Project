#include "utils.h"
#include <stdio.h>
#include <stdlib.h>

/* Unpacks the 32-bit machine code instruction given into the correct
 * type within the instruction struct */
Instruction parse_instruction(uint32_t instruction_bits) {
  /* YOUR CODE HERE */
  Instruction instruction;
  // add x9, x20, x21   hex: 01 5A 04 B3, binary = 0000 0001 0101 1010 0000 0100 1011 0011
  // Opcode: 0110011 (0x33) Get the Opcode by &ing 0x1111111, bottom 7 bits
  instruction.opcode = instruction_bits & ((1U << 7) - 1);

  // Shift right to move to pointer to interpret next fields in instruction.
  instruction_bits >>= 7;

  switch (instruction.opcode) {
  // R-Type
  case 0x33:
    // instruction: 0000 0001 0101 1010 0000 0100 1, destination : 01001
    instruction.rtype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // instruction: 0000 0001 0101 1010 0000, func3 : 000
    instruction.rtype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    // instruction: 0000 0001 0101 1010 0, src1: 10100
    instruction.rtype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // instruction: 0000 0001 0101, src2: 10101
    instruction.rtype.rs2 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    // funct7: 0000 000
    instruction.rtype.funct7 = instruction_bits & ((1U << 7) - 1);
    break;
  // cases for other types of instructions
  /* YOUR CODE HERE */

  // I-Type (Loads)
  case 0x3:
    //destination register
    instruction.itype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //func3
    instruction.itype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    //rs1
    instruction.itype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //imm
    instruction.itype.imm = instruction_bits & ((1U << 12) - 1);
    break;

  // I-Type (Non-Loads)
  case 0x13:
    //destination register
    instruction.itype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //func3
    instruction.itype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    //rs1
    instruction.itype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //imm
    instruction.itype.imm = instruction_bits & ((1U << 12) - 1);
    break;

  //S-type
  case 0x23:
    //immediate
    instruction.stype.imm5 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //funct3
    instruction.stype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    //rs1
    instruction.stype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //rs2
    instruction.stype.rs2 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //imm7
    instruction.stype.imm7 = instruction_bits & ((1U << 7) - 1);
    break;

  //B-type
  case 0x63:
    //immediate 5
    instruction.sbtype.imm5 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //funct3
    instruction.sbtype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    //rs1
    instruction.sbtype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //rs2
    instruction.sbtype.rs2 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //immediate 7
    instruction.sbtype.imm7 = instruction_bits & ((1U << 7) - 1);
    break;
  
  //U-type(LUI or AUIPC)
  case 0x37:
  //case 0x17:
    //Destination register
    instruction.utype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //Immediate
    instruction.utype.imm = instruction_bits & ((1U << 20) - 1);
    break;
  
  //J-type(JAL OR JALR)
  //case 0x67:
  case 0x6F:
    //Destination register
    instruction.ujtype.rd = instruction_bits & ((1U << 5) -1 );
    instruction_bits >>= 5;

    //Immediate
    instruction.ujtype.imm = instruction_bits & ((1U << 20) -1);
    break;
  
  //I-type(Ecall, EBreak)
  case 0x73:
    //destination register
    instruction.itype.rd = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //func3
    instruction.itype.funct3 = instruction_bits & ((1U << 3) - 1);
    instruction_bits >>= 3;

    //rs1
    instruction.itype.rs1 = instruction_bits & ((1U << 5) - 1);
    instruction_bits >>= 5;

    //imm
    instruction.itype.imm = instruction_bits & ((1U << 12) - 1);
    break;

  #ifndef TESTING
  default:
    exit(EXIT_FAILURE);
  #endif
  }
  return instruction;
}

/************************Helper functions************************/
/* Here, you will need to implement a few common helper functions, 
 * which you will call in other functions when parsing, printing, 
 * or executing the instructions. */

int sign_extend_number(unsigned int field, unsigned int n) {
    // Mask to ensure only lower 'n' bits are used
    field &= (1U << n) - 1;

    // Check if sign bit (bit n-1) is set
    if (field & (1U << (n - 1))) {
        // Negative number: extend sign by filling upper bits with 1s
        return (int)(field | ~((1U << n) - 1));
    } else {
        // Positive number: keep as is
        return (int)field;
    }
}



/* Return the number of bytes (from the current PC) to the branch label using
 * the given branch instruction */
int get_branch_offset(Instruction instruction) {
    int imm = 0;

    imm |= (instruction.sbtype.imm5 & 0x1) << 11;                         // imm[11] = imm5[0]
    imm |= ((instruction.sbtype.imm5 >> 1) & 0xF) << 1;                   // imm[4:1] = imm5[4:1]
    imm |= (instruction.sbtype.imm7 & 0x3F) << 5;                         // imm[10:5] = imm7[5:0]
    imm |= ((instruction.sbtype.imm7 >> 6) & 0x1) << 12;                  // imm[12] = imm7[6]

    return sign_extend_number(imm, 13); // Branch offsets are 13-bit signed
}


/* Returns the number of bytes (from the current PC) to the jump label using the
 * given jump instruction */
int get_jump_offset(Instruction instruction) {
  /* YOUR CODE HERE */
  //Store the immediate bits
  int imm = 0;
  unsigned int imm_part = instruction.ujtype.imm;

  //Extract imm[20], shift it to position 20
  imm |= (imm_part & 0x80000) << 1;
  //Extract imm[11]
  imm |= (imm_part & 0x00100) << 3;
  //Extract imm[10:1]
  imm |= (imm_part & 0x7FE00) >> 8;
  //Extract imm[19:12]
  imm |= (imm_part & 0xFF) << 12;

  return sign_extend_number (imm, 20);
}

/* Returns the number of bytes (from the current PC) to the base address using the
 * given store instruction */
int get_store_offset(Instruction instruction) {
  /* YOUR CODE HERE */
  //Stores the imm bits
  int imm = 0;

  //Extract the imm5 [4:0] bits from S-type
  imm |= (instruction.stype.imm5 & 0x1F);
  //Extract bits [11:5] from imm7 S-type
  imm |= (instruction.stype.imm7 & 0x7F) << 5;
  return sign_extend_number(imm, 12);
}
/************************Helper functions************************/

void handle_invalid_instruction(Instruction instruction) {
  printf("Invalid Instruction: 0x%08x\n", instruction.bits);
}

void handle_invalid_read(Address address) {
  printf("Bad Read. Address: 0x%08x\n", address);
  exit(-1);
}

void handle_invalid_write(Address address) {
  printf("Bad Write. Address: 0x%08x\n", address);
  exit(-1);
}