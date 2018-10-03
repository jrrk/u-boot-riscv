/*
 * Copyright (C) 2017 Andes Technology Corporation
 * Rick Chen, Andes Technology Corporation <rick@andestech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

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
}

void icache_disable(void)
{
  debug("icache_disable();\n");
}

int icache_status(void)
{
  uint64_t stat = 0;
  debug("icache_status() => %llx\n", stat);
  return stat;
}

void dcache_enable(void)
{
  debug("dcache_enable();\n");
}

void dcache_disable(void)
{
  debug("dcache_disable();\n");
}

int dcache_status(void)
{
  uint64_t stat = 0;
  debug("dcache_status() => %llx\n", stat);
  return 0;
}
