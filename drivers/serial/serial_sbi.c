/*
 * Copyright (c) 2015 National Instruments
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <serial.h>

static int sbi_serial_setbrg(struct udevice *dev, int baudrate)
{
	return 0;
}

static int sbi_serial_getc(struct udevice *dev)
{
	return -EAGAIN;
}

static int sbi_serial_pending(struct udevice *dev, bool input)
{
	return 0;
}

static int sbi_serial_putc(struct udevice *dev, const char ch)
{
	return 0;
}

static const struct udevice_id sbi_serial_ids[] = {
	{ .compatible = "sbi-serial" },
	{ }
};


const struct dm_serial_ops sbi_serial_ops = {
	.putc = sbi_serial_putc,
	.pending = sbi_serial_pending,
	.getc = sbi_serial_getc,
	.setbrg = sbi_serial_setbrg,
};

U_BOOT_DRIVER(serial_sbi) = {
	.name	= "serial_sbi",
	.id	= UCLASS_SERIAL,
	.of_match = sbi_serial_ids,
	.ops	= &sbi_serial_ops,
};

int sbi_serial_init(void)
{
        return 0;
}

static void sbi_serial_setbrg_(void)
{

}

static void sbi_serial_putc_(const char c)
{

}

static int sbi_serial_tstc(void)
{
        return 0;
}

static int sbi_serial_getc_(void)
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
        .setbrg = sbi_serial_setbrg_,
        .putc   = sbi_serial_putc_,
        .puts   = default_serial_puts,
        .getc   = sbi_serial_getc_,
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
