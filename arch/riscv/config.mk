#
# (C) Copyright 2011
# Julius Baxter <julius@opencores.org>
#
# SPDX-License-Identifier:	GPL-2.0+
#

ifeq ($(CROSS_COMPILE),)
CROSS_COMPILE := riscv64-unknown-elf-
endif

# r10 used for global object pointer, already set in OR32 GCC but just to be
# clear
PLATFORM_CPPFLAGS +=

CONFIG_STANDALONE_LOAD_ADDR ?= 0x40000000
