/*
 * (C) Copyright 2011, Julius Baxter <julius@opencores.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ASM_RISCV_SYSTEM_H
#define __ASM_RISCV_SYSTEM_H

#include <asm/spr-defs.h>

static inline unsigned long mfspr(unsigned long add)
{
	unsigned long ret;

	return ret;
}

static inline void mtspr(unsigned long add, unsigned long val)
{

}

#endif /* __ASM_RISCV_SYSTEM_H */
