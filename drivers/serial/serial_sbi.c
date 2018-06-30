/*
 * Copyright (c) 2015 National Instruments
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <serial.h>

volatile uint64_t *const uart_base = (uint64_t *)0x40034000;

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
  volatile int i;
  int retval, rxwrcnt, rxrdcnt;
  uint64_t stat = uart_base[0];
  uint64_t counts = uart_base[0x400];
#if 0
  debug("serial status is %llx\n", stat);
  debug("serial counts are %llx\n", counts);
#endif  
  rxrdcnt = counts & 0x7ff;
  rxwrcnt = (counts >> 16) & 0x7ff;
#if 0
  debug("rxrdcnt=%d, rxwrcnt=%d\n", rxrdcnt, rxwrcnt);
#endif  
  retval = rxrdcnt != rxwrcnt;
#if 0
  debug("sbi_serial_tstc() returned %x\n", retval);
  for (i = 0; i < 1000000; i++)
    ;
#endif  
  return retval;
}

static int sbi_serial_getc(void)
{
  int ch = 0;
  do {
    if (sbi_serial_tstc())
      {
        uart_base[0x200] = 0;
        ch = uart_base[0] & 0x7f;
      }
  } while (!ch);
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
  extern char __bss_start[], __bss_end[];
  size_t bsslen = __bss_end - __bss_start;
  memset(__bss_start, 0, bsslen);  
  uart_base[0x400] = 108; // was 217;
  early_puts("\nhello\n");
}
