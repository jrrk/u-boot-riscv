/*
 * (C) Copyright 2013 - 2015 Xilinx, Inc.
 *
 * Minion SD Host Controller Interface base on Zynq controller
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
/*
 * Copyright 2011, Marvell Semiconductor Inc.
 * Lei Wen <leiwen@marvell.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * Back ported to the 8xx platform (from the 8260 platform) by
 * Murray.Jensen@cmst.csiro.au, 27-Jan-01.
 */

#include <os.h>
#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <malloc.h>
#include <errno.h>
#include <mmc.h>
#include "minion_lib.h"

int echo = 0;

void myassert(int cond)
{
  ((cond) ? (void) (0) : __assert_fail ("cond", "drivers/mmc/minion_helper.c", 77, __PRETTY_FUNCTION__));
}

void myexit(int status)
{
  exit(status);
}

void myperror(const char *s)
{
  perror(s);
}

void myputchar(char ch)
{
   os_putc(ch);
}

void myputs(const char *str)
{
  while (*str)
    {
      myputchar(*str++);
    }
}

void myputn(unsigned n)
{
  if (n > 9) myputn(n / 10);
  myputchar(n%10 + '0');
}

void myputhex(unsigned n, unsigned width)
{
  if (width > 1) myputhex(n >> 4, width-1);
  n &= 15;
  if (n > 9) myputchar(n + 'A' - 10);
  else myputchar(n + '0');
}

int minion_sd_debug(void)
{
  char ucmd[99];
  myputs("Hello\n");
  do {
    myputchar('\n');
    cli_readline_into_buffer("dbg> ", ucmd, 0);
    minion_dispatch(ucmd);	
  } while (*ucmd != 'q');
  myputs("\nGoodbye\n");
  return 0;
}

int minion_sd_loadelf(const char *elf)
{
  edcl_loadelf(elf);
  return 0;
}

void minion_uart_reset(struct minion_uart_host *host, u8 mask)
{
	unsigned long timeout;

	/* Wait max 100 ms */
	timeout = 100;
	minion_uart_write(host, mask, MINION_UART_SOFTWARE_RESET);
	while (minion_uart_read(host, MINION_UART_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			printf("%s: Reset 0x%x never completed.\n",
			       __func__, (int)mask);
			return;
		}
		timeout--;
		udelay(1000);
	}
}

void minion_uart_cmd_done(
			  struct minion_uart_host *host,
			  uint cmd_resp_type,
			  uint cmd_response[4],
			  u32 mode,
			  int cmd,
			  struct mmc_data *data)
{
  int i, read;
  unsigned resp[10];
  queue_read_array(sd_base, 10, resp);
  if (cmd_resp_type & MMC_RSP_136) {
    /* CRC is stripped so we need to do some shifting. */
    for (i = 0; i < 4; i++) {
      cmd_response[i] = resp[3-i] << 8;
      if (i != 3)
	cmd_response[i] |= resp[2-i] >> 24;
    }
  } else {
    cmd_response[0] = resp[0];
  }
  read = mode & MINION_UART_TRNS_READ;
#if 1
  int len = 0;
  if (read || !cmd) len = sd_flush(host->start_addr, cmd ? data->blocksize*data->blocks : 0, sd_resp(9));
  if (read)
    {
      for (i = 0; i < len; i++)
	(host->start_addr)[i] = __be32_to_cpu((host->start_addr)[i]);
    }
	
#else
  int cnt = 0;
  int ready = sd_stat(0);
  int itm, discard = 0;
  while (1 & ~ready)
    {
      rx_write_fifo(0);
      itm = rx_read_fifo();
      if (read && (cnt < data->blocksize*data->blocks))
	(host->start_addr)[cnt++] = itm; else discard++;
      ready = sd_stat(0);
    }
  if (read)
    {
#ifdef LEGACY
      (host->start_addr)[cnt] = 0;
      for (i = 0; i < cnt; i++)
	(host->start_addr)[i] = ((host->start_addr)[i] << 24) | ((host->start_addr)[i+1] >> 8);
#endif
      for (i = 0; i < cnt; i++)
	(host->start_addr)[i] = __be32_to_cpu((host->start_addr)[i]);
    }      
#endif      
}

#ifdef CONFIG_DM_MMC_OPS
static int minion_uart_send_command(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);

#else
static int minion_uart_send_command(struct mmc *mmc, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
#endif
	struct minion_uart_host *host = mmc->priv;
	unsigned int stat = 0;
	int ret = 0;
	u32 mask, flags, mode = 0;
	unsigned int time = 0;
	int mmc_dev = mmc_get_blk_desc(mmc)->devnum;
	unsigned start = get_timer(0);

	/* Timeout unit - ms */
	static unsigned int cmd_timeout = MINION_UART_CMD_DEFAULT_TIMEOUT;

	minion_uart_write(host, MINION_UART_INT_ALL_MASK, MINION_UART_INT_STATUS);
	mask = MINION_UART_CMD_INHIBIT | MINION_UART_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		mask &= ~MINION_UART_DATA_INHIBIT;

	while (minion_uart_read(host, MINION_UART_PRESENT_STATE) & mask) {
		if (time >= cmd_timeout) {
			printf("%s: MMC: %d busy ", __func__, mmc_dev);
			if (2 * cmd_timeout <= MINION_UART_CMD_MAX_TIMEOUT) {
				cmd_timeout += cmd_timeout;
				printf("timeout increasing to: %u ms.\n",
				       cmd_timeout);
			} else {
				puts("timeout.\n");
				return -ECOMM;
			}
		}
		time++;
		udelay(1000);
	}

	mask = MINION_UART_INT_RESPONSE;
	switch(cmd->resp_type)
	  {
	  case 0:
	    flags = MINION_UART_CMD_RESP_NONE; break;
	  case MMC_RSP_R1:
	    flags = MINION_UART_CMD_RESP_SHORT; break;
	  case MMC_RSP_R1b:
	    flags = MINION_UART_CMD_RESP_SHORT_BUSY; break;
	  case MMC_RSP_R2:
	    flags = MINION_UART_CMD_RESP_LONG; break;
	  case MMC_RSP_R3:
	    flags = MINION_UART_CMD_RESP_SHORT_BUSY; break;
	  default:
	    flags = 0;
	  }

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= MINION_UART_CMD_CRC;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= MINION_UART_CMD_INDEX;
	if (data)
		flags |= MINION_UART_CMD_DATA;

	/* Set Transfer mode regarding to data flag */
	if (data != 0) {
		minion_uart_write(host, 500, MINION_UART_TIMEOUT_CONTROL);
		mode = MINION_UART_TRNS_BLK_CNT_EN;
		if (data->blocks > 1)
			mode |= MINION_UART_TRNS_MULTI;

		if (data->flags == MMC_DATA_READ)
			mode |= MINION_UART_TRNS_READ;

		if (data->flags == MMC_DATA_READ)
		  host->start_addr = (uint32_t *)(data->dest);
		else
		  host->start_addr = (uint32_t *)(data->src);

		minion_uart_write(host, data->blocksize, MINION_UART_BLOCK_SIZE);
		minion_uart_write(host, data->blocks, MINION_UART_BLOCK_COUNT);
		minion_uart_write(host, mode, MINION_UART_TRANSFER_MODE);
	} else if (cmd->resp_type & MMC_RSP_BUSY) {
		minion_uart_write(host, 0xe, MINION_UART_TIMEOUT_CONTROL);
	}

	minion_uart_write(host, cmd->cmdarg, MINION_UART_ARGUMENT);
	minion_uart_write(host, MINION_UART_MAKE_CMD(cmd->cmdidx, flags), MINION_UART_COMMAND);
	start = get_timer(0);
	do {
		stat = minion_uart_read(host, MINION_UART_INT_STATUS);
		unsigned end = get_timer(start);
#ifdef CONFIG_MINION_VERBOSE
		printf("start=%d, end=%d, ((%X & %X) != %X)\n", start, end, stat, mask, mask);
#endif
		if (stat & MINION_UART_INT_ERROR)
			break;

		if (end >= MINION_UART_READ_STATUS_TIMEOUT) {
		  {
				printf("%s: Timeout for status update!\n",
				       __func__);
				return -ETIMEDOUT;
			}
		}
	} while ((stat & mask) != mask);

	if ((stat & (MINION_UART_INT_ERROR | mask)) == mask) {
	  minion_uart_cmd_done(host, cmd->resp_type, cmd->response, mode, cmd->cmdidx, data);
		minion_uart_write(host, mask, MINION_UART_INT_STATUS);
	} else
		ret = -1;

	stat = minion_uart_read(host, MINION_UART_INT_STATUS);
	minion_uart_write(host, MINION_UART_INT_ALL_MASK, MINION_UART_INT_STATUS);
	if (!ret) {
		return 0;
	}

	minion_uart_reset(host, MINION_UART_RESET_CMD);
	minion_uart_reset(host, MINION_UART_RESET_DATA);
	if (stat & MINION_UART_INT_TIMEOUT)
		return -ETIMEDOUT;
	else
		return -ECOMM;
}

static int minion_uart_set_clock(struct mmc *mmc, unsigned int clock)
{
	struct minion_uart_host *host = mmc->priv;
	unsigned int clk = 0, timeout, reg;

	/* Wait max 20 ms */
	timeout = 200;
	while (minion_uart_read(host, MINION_UART_PRESENT_STATE) &
			   (MINION_UART_CMD_INHIBIT | MINION_UART_DATA_INHIBIT)) {
		if (timeout == 0) {
			printf("%s: Timeout to wait cmd & data inhibit\n",
			       __func__);
			return -EBUSY;
		}

		timeout--;
		udelay(100);
	}

	reg = minion_uart_read(host, MINION_UART_CLOCK_CONTROL);
	reg &= ~(MINION_UART_CLOCK_CARD_EN | MINION_UART_CLOCK_INT_EN);
	minion_uart_write(host, reg, MINION_UART_CLOCK_CONTROL);

	host->clock = clock;
	
	if (clock == 0)
		return 0;

	clk |= (clock & MINION_UART_DIV_MASK) << MINION_UART_DIVIDER_SHIFT;
	clk |= MINION_UART_CLOCK_INT_EN;
	minion_uart_write(host, clk, MINION_UART_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = minion_uart_read(host, MINION_UART_CLOCK_CONTROL))
		& MINION_UART_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printf("%s: Internal clock never stabilised.\n",
			       __func__);
			return -EBUSY;
		}
		timeout--;
		udelay(1000);
	}

	clk |= MINION_UART_CLOCK_CARD_EN;
	minion_uart_write(host, clk, MINION_UART_CLOCK_CONTROL);
	return 0;
}

static void minion_uart_set_power(struct minion_uart_host *host, unsigned short power)
{
	u8 pwr = 0;

	if (power != (unsigned short)-1) {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = MINION_UART_POWER_180;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = MINION_UART_POWER_300;
			break;
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = MINION_UART_POWER_330;
			break;
		}
	}

	if (pwr == 0) {
		minion_uart_write(host, 0, MINION_UART_POWER_CONTROL);
		return;
	}

	pwr |= MINION_UART_POWER_ON;

	minion_uart_write(host, pwr, MINION_UART_POWER_CONTROL);
}

#ifdef CONFIG_DM_MMC_OPS
static int minion_uart_set_ios(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static void minion_uart_set_ios(struct mmc *mmc)
{
#endif
	u32 ctrl;
	struct minion_uart_host *host = mmc->priv;

	if (mmc->clock != host->clock)
		minion_uart_set_clock(mmc, mmc->clock);

	/* Set bus width */
	ctrl = minion_uart_read(host, MINION_UART_HOST_CONTROL);
	if (mmc->bus_width == 8) {
		ctrl &= ~MINION_UART_CTRL_4BITBUS;
		if ((MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300))
			ctrl |= MINION_UART_CTRL_8BITBUS;
	} else {
		if ((MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300))
			ctrl &= ~MINION_UART_CTRL_8BITBUS;
		if (mmc->bus_width == 4)
			ctrl |= MINION_UART_CTRL_4BITBUS;
		else
			ctrl &= ~MINION_UART_CTRL_4BITBUS;
	}

	if (mmc->clock > 26000000)
		ctrl |= MINION_UART_CTRL_HISPD;
	else
		ctrl &= ~MINION_UART_CTRL_HISPD;

	minion_uart_write(host, ctrl, MINION_UART_HOST_CONTROL);
#ifdef CONFIG_DM_MMC_OPS
	return 0;
#endif
}

const struct dm_mmc_ops minion_uart_ops = {
	.send_cmd	= minion_uart_send_command,
	.set_ios	= minion_uart_set_ios,
};

 int minion_uart_setup_cfg(struct mmc_config *cfg, struct minion_uart_host *host)
{
	u32 caps;

	caps = minion_uart_read(host, MINION_UART_CAPABILITIES);

	host->version = minion_uart_read(host, MINION_UART_HOST_VERSION);
	
	cfg->name = host->name;
	cfg->f_max = 800000;
	cfg->f_min = 100000;
	
#ifndef CONFIG_DM_MMC_OPS
	cfg->ops = &minion_uart_ops;
#endif
	cfg->voltages = 0;
	if (caps & MINION_UART_CAN_VDD_330)
		cfg->voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (caps & MINION_UART_CAN_VDD_300)
		cfg->voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	if (caps & MINION_UART_CAN_VDD_180)
		cfg->voltages |= MMC_VDD_165_195;

	cfg->host_caps = MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_4BIT;
	if (MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300) {
		if (caps & MINION_UART_CAN_DO_8BIT)
			cfg->host_caps |= MMC_MODE_8BIT;
	}

	if (host->host_caps)
		cfg->host_caps |= host->host_caps;


	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

	return 0;
}

struct minion_uart_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

static int minion_uart_probe(struct udevice *dev)
{
	struct minion_uart_plat *plat = dev_get_platdata(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct minion_uart_host *host = dev_get_priv(dev);
	int ret;

	host->name = "minion_uart";

	ret = minion_uart_setup_cfg(&plat->cfg, host);

	host->mmc = &plat->mmc;
	if (ret)
		return ret;
	host->mmc->priv = host;
	host->mmc->dev = dev;
	upriv->mmc = host->mmc;

	struct mmc *mmc = upriv->mmc;

	minion_uart_reset(host, MINION_UART_RESET_ALL);

	minion_uart_set_power(host, fls(mmc->cfg->voltages) - 1);

	/* Enable only interrupts served by the SD controller */
	minion_uart_write(host, MINION_UART_INT_DATA_MASK | MINION_UART_INT_CMD_MASK,
		     MINION_UART_INT_ENABLE);
	/* Mask all uart interrupt sources */
	minion_uart_write(host, 0x0, MINION_UART_SIGNAL_ENABLE);

	//	list_add_tail(&mmc->link, &mmc_devices);

	return 0;
}

static int minion_uart_ofdata_to_platdata(struct udevice *dev)
{
	struct minion_uart_host *host = dev_get_priv(dev);

	host->name = dev->name;

	return 0;
}

static int minion_uart_bind(struct udevice *dev)
{
	struct minion_uart_plat *plat = dev_get_platdata(dev);
	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id minion_uart_ids[] = {
	{ }
};

U_BOOT_DRIVER(minion_uart_drv) = {
	.name		= "minion_uart",
	.id		= UCLASS_MMC,
	.of_match	= minion_uart_ids,
	.ofdata_to_platdata = minion_uart_ofdata_to_platdata,
	.ops		= &minion_uart_ops,
	.bind		= minion_uart_bind,
	.probe		= minion_uart_probe,
	.priv_auto_alloc_size = sizeof(struct minion_uart_host),
	.platdata_auto_alloc_size = sizeof(struct minion_uart_plat),
};

 static struct minion_uart_plat minion_plat;

 U_BOOT_DEVICE(minion_device) = {
   .name = "minion_uart",
   .platdata = &minion_plat,
 };
