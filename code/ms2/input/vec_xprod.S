main:
    # Load base addresses and N
    addi    x1, x0, 16              # jBlk = 16
    slli    x2, x1, 9               # Base address of array uvw = 8192
    add     x3, x2, x2              # Base address of array xyz = 16384
    add     x4, x2, x3              # Base address of array result = 24576
    addi    x5, x0, 256             # N = 256

    # Initialize input memory to contain values
    add     x6, x0, x2              # init_loop var
init_loop:
    sw      x6, 0(x6)               # Store value at address x10
    addi    x6, x6, 4               # Increment address
    bne     x6, x4, init_loop       # Loop if loop var < x4

    # Initialize loop counters
    addi    x9, x0, 0               # jj loop counter
    addi    x10, x0, 0              # i loop counter
    addi    x11, x0, 0              # j loop counter

loop_jj:
    beq     x9, x5, end         # if jj >= N, end
    addi    x10, x0, 0          # Reset i loop counter

loop_i:
    beq     x10, x5, loop_jj_end # if i >= N, end loop_i
    # Initialize temp_result to zero
    addi    x6, x0, 0
    addi    x7, x0, 0
    addi    x8, x0, 0

    # Calculate i offest
    slli    x12, x10, 2          # x12 = i * 4
    slli    x13, x12, 1          # x13 = (i * 4) * 2
    add     x13, x13, x12        # x13 = (i * 4) * 2 + (i * 4) = (i * 4) * 3
    # Load address of uwv[i]
    add     x12, x2, x13         # x12 = uvw + (i * 4) * 3
    # Load address of xyz[i]
    add     x13, x3, x13         # x13 = xyz + (i * 4) * 3

    # Load uwv[i][0-2]
    lw      x14, 0(x12)
    lw      x15, 4(x12)
    lw      x16, 8(x12)

    # Load xyz[i][0-2]
    lw      x17, 0(x13)
    lw      x18, 4(x13)
    lw      x19, 8(x13)

    addi    x11, x9, 0           # j = jj
    add     x20, x9, x1          # x20 = jj + jBlk
loop_j:
    beq     x11, x20, loop_i_end  # if j >= jj + jBlk, end loop_j
    beq     x10, x11, loop_j_inc # if i == j, skip this iteration

    # Calculate j offest
    slli    x21, x11, 2          # x21 = j * 4
    slli    x22, x21, 1          # x22 = (j * 4) * 2
    add     x22, x22, x21        # x22 = (j * 4) * 2 + (j * 4) = (j * 4) * 3
    # Load address of xyz[j]
    add     x21, x3, x22         # x21 = xyz + (j * 4) * 3

    lw      x22, 0(x21)         # x22 = xyz[j][0]
    sub     x22, x22, x17       # r[0] = xyz[j][0] - xyz[i][0]

    lw      x23, 4(x21)         # x23 = xyz[j][1]
    sub     x23, x23, x18       # r[1] = xyz[j][1] - xyz[i][1]

    lw      x24, 8(x21)         # x24 = xyz[j][2]
    sub     x24, x24, x19       # r[2] = xyz[j][2] - xyz[i][2]

    # Compute cross product: a = uvw[i], b = r
    mul     x25, x15, x24       # a[1] * b[2]
    mul     x26, x16, x23       # a[2] * b[1]
    sub     x27, x25, x26       # result[0] = a[1] * b[2] - a[2] * b[1]

    mul     x25, x16, x22       # a[2] * b[0]
    mul     x26, x14, x24       # a[0] * b[2]
    sub     x28, x25, x26       # result[1] = a[2] * b[0] - a[0] * b[2]

    mul     x25, x14, x23       # a[0] * b[1]
    mul     x26, x15, x22       # a[1] * b[0]
    sub     x29, x25, x26       # result[2] = a[0] * b[1] - a[1] * b[0]

    # Accumulate temporary result
    add     x6, x6, x27         # temp_result[0] += result[0]
    add     x7, x7, x28         # temp_result[1] += result[1]
    add     x8, x8, x29         # temp_result[2] += result[2]

loop_j_inc:
    addi    x11, x11, 1         # j++
    j       loop_j              # Repeat j loop

loop_i_end:
    # Store accumulated result in result[i]
    add     x30, x2, x13        # x30 = result + (i * 4) * 3

    lw      x27, 0(x30)
    add     x6, x6, x27
    sw      x6, 0(x30)          # result[i][0] += temp_result[0]

    lw      x28, 4(x30)
    add     x7, x7, x28
    sw      x7, 4(x30)          # result[i][1] += temp_result[1]

    lw      x29, 8(x30)
    add     x8, x8, x29
    sw      x8, 8(x30)          # result[i][2] += temp_result[2]

    addi    x10, x10, 1         # i++
    j       loop_i              # Repeat i loop

loop_jj_end:
    add     x9, x9, x1          # jj += jBlk
    j       loop_jj             # Repeat jj loop

end:
    addi   x10, x0, 10
    ecall
