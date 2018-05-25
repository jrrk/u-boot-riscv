/*
 * Copyright (C) 2017 Andes Technology Corporation
 * Rick Chen, Andes Technology Corporation <rick@andestech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#define cache_addr 0x702

#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define swap_csr(reg, val) ({ unsigned long __tmp; \
  asm volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "rK"(val)); \
  __tmp; })

#define set_csr(reg, bit) ({ unsigned long __tmp; \
  asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "rK"(bit)); \
  __tmp; })

#define clear_csr(reg, bit) ({ unsigned long __tmp; \
  asm volatile ("csrrc %0, " #reg ", %1" : "=r"(__tmp) : "rK"(bit)); \
  __tmp; })

void flush_dcache_range(unsigned long start, unsigned long end)
{
}

void invalidate_icache_range(unsigned long start, unsigned long end)
{
}

void invalidate_dcache_range(unsigned long start, unsigned long end)
{
}

void flush_cache(unsigned long addr, unsigned long size)
{
}

void icache_enable(void)
{
}

void icache_disable(void)
{
}

int icache_status(void)
{
	return 1;
}

void dcache_enable(void)
{
  debug("dcache_enable();\n");
//  write_csr(0x702, 0x80000000);
  write_csr(0x701, 0x1);
  debug("dcache enabled\n");
}

void dcache_disable(void)
{
  debug("dcache_disable();\n");
//  write_csr(0x702, 0x90000000);
  asm volatile ("fence");
  write_csr(0x701, 0x0);
  debug("dcache disabled\n");
}

int dcache_status(void)
{
//  unsigned long stat = read_csr(0x702);
  unsigned long stat = read_csr(0x701);
  debug("dcache_status() => %lx\n", stat);
  return stat;
}
