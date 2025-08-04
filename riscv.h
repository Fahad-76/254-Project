#ifndef MIPS_H
#define MIPS_H

#include <stdbool.h>
#include "types.h"

/* see disasm.c */
void decode_instruction(uint32_t instruction_bits);

/* see emulator.c */
void execute_instruction(uint32_t instruction_bits, regfile_t* regfile, Byte *memory);
void store(Byte *memory, Address address, Alignment alignment, Word value);
Word load(Byte *memory, Address address, Alignment alignment);

// Settings for cycle accurate simulator
typedef struct
{
    bool cache_en;
    bool fwd_en;
}simulator_config_t;

#endif
