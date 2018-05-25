/*
 * Copyright (C) 2017 Andes Technology Corporation
 * Rick Chen, Andes Technology Corporation <rick@andestech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#define read_csr(reg) ({ uint64_t __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define swap_csr(reg, val) ({ uint64_t __tmp; \
  asm volatile ("csrrw %0, " #reg ", %1" : "=r"(__tmp) : "rK"(val)); \
  __tmp; })

#define set_csr(reg, bit) ({ uint64_t __tmp; \
  asm volatile ("csrrs %0, " #reg ", %1" : "=r"(__tmp) : "rK"(bit)); \
  __tmp; })

#define clear_csr(reg, bit) ({ uint64_t __tmp; \
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
  debug("icache_enable();\n");
  asm volatile ("fence.i");
  write_csr(0x701, 0x1);
  asm volatile ("fence.i");
  debug("icache enabled\n");
}

void icache_disable(void)
{
  debug("icache_disable();\n");
  asm volatile ("fence.i");
  write_csr(0x700, 0x0);
  asm volatile ("fence.i");
  debug("icache disabled\n");
}

int icache_status(void)
{
  uint64_t stat = read_csr(0x700);
  debug("icache_status() => %llx\n", stat);
  return stat;
}

void dcache_enable(void)
{
  debug("dcache_enable();\n");
  asm volatile ("fence");
  write_csr(0x701, 0x0);
  write_csr(0x702, 0x80000000ULL);
  write_csr(0x701, 0x1);
  debug("dcache enabled\n");
}

void dcache_disable(void)
{
  debug("dcache_disable();\n");
  asm volatile ("fence");
  write_csr(0x701, 0x0);
  write_csr(0x702, 0xFFFFF000ULL);
  write_csr(0x701, 0x1);
  debug("dcache disabled\n");
}

int dcache_status(void)
{
  uint64_t stat = read_csr(0x702);
  debug("dcache_status() => %llx\n", stat);
  return stat < 0xFFFFF000ULL;
}
