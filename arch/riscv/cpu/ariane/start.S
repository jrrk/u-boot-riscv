/*
 * Startup Code for RISC-V Core
 *
 * Copyright (c) 2017 Microsemi Corporation.
 * Copyright (c) 2017 Padmarao Begari <Padmarao.Begari@microsemi.com>
 *
 * Copyright (C) 2017 Andes Technology Corporation
 * Rick Chen, Andes Technology Corporation <rick@andestech.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <asm-offsets.h>
#include <config.h>
#include <common.h>
#include <elf.h>
#include <asm/encoding.h>

#ifdef CONFIG_32BIT
#define LREG 			lw
#define SREG 			sw
#define REGBYTES 		4
#define RELOC_TYPE		R_RISCV_32
#define SYM_INDEX		0x8
#define SYM_SIZE		0x10
#else
#define LREG 			ld
#define SREG 			sd
#define REGBYTES 		8
#define RELOC_TYPE		R_RISCV_64
#define SYM_INDEX		0x20
#define SYM_SIZE		0x18
#endif

.section      .text
.globl _start
_start:
	j handle_reset

nmi_vector:
	j nmi_vector

trap_vector:
	j trap_entry

.global trap_entry
handle_reset:
	la t0, trap_entry
	csrw mtvec, t0
	csrwi mstatus, 0
	csrwi mie, 0
	li  t0, -16
	li  t1, CONFIG_SYS_INIT_SP_ADDR
	and sp, t1, t0	/* force 16 byte alignment */
	la  t0, __bss_start
	la  t1, __bss_end
1:	sw  zero, (t0)
	addi t0, t0, 4
	blt t0, t1, 1b
	
        jal     early_uart_init
	mv	a0, sp
	jal	board_init_f_alloc_reserve
	mv	sp, a0
	jal	board_init_f_init_reserve

	mv  a0, zero	/* a0 <-- boot_flags = 0 */
	la t5, board_init_f
	jr t5		/* jump to board_init_r() */

/*
 * trap entry
 */
trap_entry:
	addi sp, sp, -32*REGBYTES
	SREG x1, 1*REGBYTES(sp)
	SREG x2, 2*REGBYTES(sp)
	SREG x3, 3*REGBYTES(sp)
	SREG x4, 4*REGBYTES(sp)
	SREG x5, 5*REGBYTES(sp)
	SREG x6, 6*REGBYTES(sp)
	SREG x7, 7*REGBYTES(sp)
	SREG x8, 8*REGBYTES(sp)
	SREG x9, 9*REGBYTES(sp)
	SREG x10, 10*REGBYTES(sp)
	SREG x11, 11*REGBYTES(sp)
	SREG x12, 12*REGBYTES(sp)
	SREG x13, 13*REGBYTES(sp)
	SREG x14, 14*REGBYTES(sp)
	SREG x15, 15*REGBYTES(sp)
	SREG x16, 16*REGBYTES(sp)
	SREG x17, 17*REGBYTES(sp)
	SREG x18, 18*REGBYTES(sp)
	SREG x19, 19*REGBYTES(sp)
	SREG x20, 20*REGBYTES(sp)
	SREG x21, 21*REGBYTES(sp)
	SREG x22, 22*REGBYTES(sp)
	SREG x23, 23*REGBYTES(sp)
	SREG x24, 24*REGBYTES(sp)
	SREG x25, 25*REGBYTES(sp)
	SREG x26, 26*REGBYTES(sp)
	SREG x27, 27*REGBYTES(sp)
	SREG x28, 28*REGBYTES(sp)
	SREG x29, 29*REGBYTES(sp)
	SREG x30, 30*REGBYTES(sp)
	SREG x31, 31*REGBYTES(sp)
	csrr a0, mcause
	csrr a1, mepc
	mv a2, sp
	jal handle_trap
	csrw mepc, a0

/*
 * Remain in M-mode after mret
 */
	li t0, MSTATUS_MPP
	csrs mstatus, t0
	LREG x1, 1*REGBYTES(sp)
	LREG x2, 2*REGBYTES(sp)
	LREG x3, 3*REGBYTES(sp)
	LREG x4, 4*REGBYTES(sp)
	LREG x5, 5*REGBYTES(sp)
	LREG x6, 6*REGBYTES(sp)
	LREG x7, 7*REGBYTES(sp)
	LREG x8, 8*REGBYTES(sp)
	LREG x9, 9*REGBYTES(sp)
	LREG x10, 10*REGBYTES(sp)
	LREG x11, 11*REGBYTES(sp)
	LREG x12, 12*REGBYTES(sp)
	LREG x13, 13*REGBYTES(sp)
	LREG x14, 14*REGBYTES(sp)
	LREG x15, 15*REGBYTES(sp)
	LREG x16, 16*REGBYTES(sp)
	LREG x17, 17*REGBYTES(sp)
	LREG x18, 18*REGBYTES(sp)
	LREG x19, 19*REGBYTES(sp)
	LREG x20, 20*REGBYTES(sp)
	LREG x21, 21*REGBYTES(sp)
	LREG x22, 22*REGBYTES(sp)
	LREG x23, 23*REGBYTES(sp)
	LREG x24, 24*REGBYTES(sp)
	LREG x25, 25*REGBYTES(sp)
	LREG x26, 26*REGBYTES(sp)
	LREG x27, 27*REGBYTES(sp)
	LREG x28, 28*REGBYTES(sp)
	LREG x29, 29*REGBYTES(sp)
	LREG x30, 30*REGBYTES(sp)
	LREG x31, 31*REGBYTES(sp)
	addi sp, sp, 32*REGBYTES
	mret

#ifdef CONFIG_INIT_CRITICAL
cpu_init_crit:
    ret
#endif
