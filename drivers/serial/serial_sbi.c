/*
 * Copyright (c) 2015 National Instruments
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <serial.h>

volatile uint64_t *const uart_base = (uint64_t *)0x41004000;

int sbi_serial_init(void)
{
        return 0;
}

static void sbi_serial_setbrg(void)
{

}

static void sbi_serial_putc(const char c)
{
  while (uart_base[0] & 0x400)
    ;
  uart_base[0] = c;
}

static int sbi_serial_tstc(void)
{
        return 0;
}

static int sbi_serial_getc(void)
{
  int ch = 0;
        while (1) {

                }

        return ch;
}

static struct serial_device sbi_serial_drv = {
        .name   = "sbi_serial",
        .start  = sbi_serial_init,
        .stop   = NULL,
        .setbrg = sbi_serial_setbrg,
        .putc   = sbi_serial_putc,
        .puts   = default_serial_puts,
        .getc   = sbi_serial_getc,
        .tstc   = sbi_serial_tstc,
};

void sbi_serial_initialize(void)
{
        serial_register(&sbi_serial_drv);
}

__weak struct serial_device *default_serial_console(void)
{
        return &sbi_serial_drv;
}

void early_puts(const char *msg)
{
  while (*msg) sbi_serial_putc(*msg++);
}

void early_uart_init(void)
{
  uart_base[0x400] = 54; // was 217;
  early_puts("\nhello\n");
}
