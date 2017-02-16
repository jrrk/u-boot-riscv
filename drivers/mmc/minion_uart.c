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

void sdhci_reset(struct minion_uart_host *host, u8 mask)
{
	unsigned long timeout;

	/* Wait max 100 ms */
	timeout = 100;
	sdhci_write(host, mask, SDHCI_SOFTWARE_RESET);
	while (sdhci_read(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0) {
			printf("%s: Reset 0x%x never completed.\n",
			       __func__, (int)mask);
			return;
		}
		timeout--;
		udelay(1000);
	}
}

uint32_t to_cpu(uint32_t arg)
{
  return __be32_to_cpu(arg);
}

void sdhci_cmd_done(
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
  if (mode & SDHCI_TRNS_READ)
	memcpy(host->start_addr, minion_iobuf, data->blocksize*data->blocks);
}

#ifdef CONFIG_DM_MMC_OPS
static int sdhci_send_command(struct udevice *dev, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);

#else
static int sdhci_send_command(struct mmc *mmc, struct mmc_cmd *cmd,
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
	static unsigned int cmd_timeout = SDHCI_CMD_DEFAULT_TIMEOUT;

	sdhci_write(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	mask = SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT;

	/* We shouldn't wait for data inihibit for stop commands, even
	   though they might use busy signaling */
	if (cmd->cmdidx == MMC_CMD_STOP_TRANSMISSION)
		mask &= ~SDHCI_DATA_INHIBIT;

	while (sdhci_read(host, SDHCI_PRESENT_STATE) & mask) {
		if (time >= cmd_timeout) {
			printf("%s: MMC: %d busy ", __func__, mmc_dev);
			if (2 * cmd_timeout <= SDHCI_CMD_MAX_TIMEOUT) {
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

	mask = SDHCI_INT_RESPONSE;
	switch(cmd->resp_type)
	  {
	  case 0:
	    flags = SDHCI_CMD_RESP_NONE; break;
	  case MMC_RSP_R1:
	    flags = SDHCI_CMD_RESP_SHORT; break;
	  case MMC_RSP_R1b:
	    flags = SDHCI_CMD_RESP_SHORT_BUSY; break;
	  case MMC_RSP_R2:
	    flags = SDHCI_CMD_RESP_LONG; break;
	  case MMC_RSP_R3:
	    flags = SDHCI_CMD_RESP_SHORT_BUSY; break;
	  default:
	    flags = 0;
	  }

	if (cmd->resp_type & MMC_RSP_CRC)
		flags |= SDHCI_CMD_CRC;
	if (cmd->resp_type & MMC_RSP_OPCODE)
		flags |= SDHCI_CMD_INDEX;
	if (data)
		flags |= SDHCI_CMD_DATA;

	/* Set Transfer mode regarding to data flag */
	if (data != 0) {
		sdhci_write(host, 500, SDHCI_TIMEOUT_CONTROL);
		mode = SDHCI_TRNS_BLK_CNT_EN;
		if (data->blocks > 1)
			mode |= SDHCI_TRNS_MULTI;

		if (data->flags == MMC_DATA_READ)
			mode |= SDHCI_TRNS_READ;

		if (data->flags == MMC_DATA_READ)
		  host->start_addr = (uint32_t *)(data->dest);
		else
		  host->start_addr = (uint32_t *)(data->src);

		sdhci_write(host, data->blocksize, SDHCI_BLOCK_SIZE);
		sdhci_write(host, data->blocks, SDHCI_BLOCK_COUNT);
		sdhci_write(host, mode, SDHCI_TRANSFER_MODE);
	} else if (cmd->resp_type & MMC_RSP_BUSY) {
		sdhci_write(host, 0xe, SDHCI_TIMEOUT_CONTROL);
	}

	sdhci_write(host, cmd->cmdarg, SDHCI_ARGUMENT);
	sdhci_write(host, SDHCI_MAKE_CMD(cmd->cmdidx, flags), SDHCI_COMMAND);
	start = get_timer(0);
	do {
		stat = sdhci_read(host, SDHCI_INT_STATUS);
		unsigned end = get_timer(start);
#ifdef CONFIG_MINION_VERBOSE
		printf("start=%d, end=%d, ((%X & %X) != %X)\n", start, end, stat, mask, mask);
#endif
		if (stat & SDHCI_INT_ERROR)
			break;

		if (end >= SDHCI_READ_STATUS_TIMEOUT) {
		  {
				printf("%s: Timeout for status update!\n",
				       __func__);
				return -ETIMEDOUT;
			}
		}
	} while ((stat & mask) != mask);

	if ((stat & (SDHCI_INT_ERROR | mask)) == mask) {
	  sdhci_cmd_done(host, cmd->resp_type, cmd->response, mode, cmd->cmdidx, data);
		sdhci_write(host, mask, SDHCI_INT_STATUS);
	} else
		ret = -1;

	stat = sdhci_read(host, SDHCI_INT_STATUS);
	sdhci_write(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
	if (!ret) {
		return 0;
	}

	sdhci_reset(host, SDHCI_RESET_CMD);
	sdhci_reset(host, SDHCI_RESET_DATA);
	if (stat & SDHCI_INT_TIMEOUT)
		return -ETIMEDOUT;
	else
		return -ECOMM;
}

static int sdhci_set_clock(struct mmc *mmc, unsigned int clock)
{
	struct minion_uart_host *host = mmc->priv;
	unsigned int clk = 0, timeout, reg;

	/* Wait max 20 ms */
	timeout = 200;
	while (sdhci_read(host, SDHCI_PRESENT_STATE) &
			   (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		if (timeout == 0) {
			printf("%s: Timeout to wait cmd & data inhibit\n",
			       __func__);
			return -EBUSY;
		}

		timeout--;
		udelay(100);
	}

	reg = sdhci_read(host, SDHCI_CLOCK_CONTROL);
	reg &= ~(SDHCI_CLOCK_CARD_EN | SDHCI_CLOCK_INT_EN);
	sdhci_write(host, reg, SDHCI_CLOCK_CONTROL);

	host->clock = clock;
	
	if (clock == 0)
		return 0;

	clk |= (clock & SDHCI_DIV_MASK) << SDHCI_DIVIDER_SHIFT;
	clk |= SDHCI_CLOCK_INT_EN;
	sdhci_write(host, clk, SDHCI_CLOCK_CONTROL);

	/* Wait max 20 ms */
	timeout = 20;
	while (!((clk = sdhci_read(host, SDHCI_CLOCK_CONTROL))
		& SDHCI_CLOCK_INT_STABLE)) {
		if (timeout == 0) {
			printf("%s: Internal clock never stabilised.\n",
			       __func__);
			return -EBUSY;
		}
		timeout--;
		udelay(1000);
	}

	clk |= SDHCI_CLOCK_CARD_EN;
	sdhci_write(host, clk, SDHCI_CLOCK_CONTROL);
	return 0;
}

static void sdhci_set_power(struct minion_uart_host *host, unsigned short power)
{
	u8 pwr = 0;

	if (power != (unsigned short)-1) {
		switch (1 << power) {
		case MMC_VDD_165_195:
			pwr = SDHCI_POWER_180;
			break;
		case MMC_VDD_29_30:
		case MMC_VDD_30_31:
			pwr = SDHCI_POWER_300;
			break;
		case MMC_VDD_32_33:
		case MMC_VDD_33_34:
			pwr = SDHCI_POWER_330;
			break;
		}
	}

	if (pwr == 0) {
		sdhci_write(host, 0, SDHCI_POWER_CONTROL);
		return;
	}

	pwr |= SDHCI_POWER_ON;

	sdhci_write(host, pwr, SDHCI_POWER_CONTROL);
}

#ifdef CONFIG_DM_MMC_OPS
static int sdhci_set_ios(struct udevice *dev)
{
	struct mmc *mmc = mmc_get_mmc_dev(dev);
#else
static void sdhci_set_ios(struct mmc *mmc)
{
#endif
	u32 ctrl;
	struct minion_uart_host *host = mmc->priv;

	if (mmc->clock != host->clock)
		sdhci_set_clock(mmc, mmc->clock);

	/* Set bus width */
	ctrl = sdhci_read(host, SDHCI_HOST_CONTROL);
	if (mmc->bus_width == 8) {
		ctrl &= ~SDHCI_CTRL_4BITBUS;
		if ((SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300))
			ctrl |= SDHCI_CTRL_8BITBUS;
	} else {
		if ((SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300))
			ctrl &= ~SDHCI_CTRL_8BITBUS;
		if (mmc->bus_width == 4)
			ctrl |= SDHCI_CTRL_4BITBUS;
		else
			ctrl &= ~SDHCI_CTRL_4BITBUS;
	}

	if (mmc->clock > 26000000)
		ctrl |= SDHCI_CTRL_HISPD;
	else
		ctrl &= ~SDHCI_CTRL_HISPD;

	sdhci_write(host, ctrl, SDHCI_HOST_CONTROL);
#ifdef CONFIG_DM_MMC_OPS
	return 0;
#endif
}

const struct dm_mmc_ops sdhci_ops = {
	.send_cmd	= sdhci_send_command,
	.set_ios	= sdhci_set_ios,
};

 int sdhci_setup_cfg(struct mmc_config *cfg, struct minion_uart_host *host)
{
	u32 caps;

	caps = sdhci_read(host, SDHCI_CAPABILITIES);

	host->version = sdhci_read(host, SDHCI_HOST_VERSION);
	
	cfg->name = host->name;
	cfg->f_max = 800000;
	cfg->f_min = 100000;
	
#ifndef CONFIG_DM_MMC_OPS
	cfg->ops = &sdhci_ops;
#endif
	cfg->voltages = 0;
	if (caps & SDHCI_CAN_VDD_330)
		cfg->voltages |= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (caps & SDHCI_CAN_VDD_300)
		cfg->voltages |= MMC_VDD_29_30 | MMC_VDD_30_31;
	if (caps & SDHCI_CAN_VDD_180)
		cfg->voltages |= MMC_VDD_165_195;

	cfg->host_caps = MMC_MODE_HS | MMC_MODE_HS_52MHz | MMC_MODE_4BIT;
	if (SDHCI_GET_VERSION(host) >= SDHCI_SPEC_300) {
		if (caps & SDHCI_CAN_DO_8BIT)
			cfg->host_caps |= MMC_MODE_8BIT;
	}

	if (host->host_caps)
		cfg->host_caps |= host->host_caps;


	cfg->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

	return 0;
}

struct sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

static int sdhci_probe(struct udevice *dev)
{
	struct sdhci_plat *plat = dev_get_platdata(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct minion_uart_host *host = dev_get_priv(dev);
	int ret;

	host->name = "sdhci";

	ret = sdhci_setup_cfg(&plat->cfg, host);

	host->mmc = &plat->mmc;
	if (ret)
		return ret;
	host->mmc->priv = host;
	host->mmc->dev = dev;
	upriv->mmc = host->mmc;

	struct mmc *mmc = upriv->mmc;

	sdhci_reset(host, SDHCI_RESET_ALL);

	sdhci_set_power(host, fls(mmc->cfg->voltages) - 1);

	/* Enable only interrupts served by the SD controller */
	sdhci_write(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK,
		     SDHCI_INT_ENABLE);
	/* Mask all uart interrupt sources */
	sdhci_write(host, 0x0, SDHCI_SIGNAL_ENABLE);

	//	list_add_tail(&mmc->link, &mmc_devices);

	return 0;
}

static int sdhci_ofdata_to_platdata(struct udevice *dev)
{
	struct minion_uart_host *host = dev_get_priv(dev);

	host->name = dev->name;

	return 0;
}

static int sdhci_bind(struct udevice *dev)
{
	struct sdhci_plat *plat = dev_get_platdata(dev);
	return mmc_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct udevice_id sdhci_ids[] = {
	{ }
};

U_BOOT_DRIVER(sdhci_drv) = {
	.name		= "sdhci",
	.id		= UCLASS_MMC,
	.of_match	= sdhci_ids,
	.ofdata_to_platdata = sdhci_ofdata_to_platdata,
	.ops		= &sdhci_ops,
	.bind		= sdhci_bind,
	.probe		= sdhci_probe,
	.priv_auto_alloc_size = sizeof(struct minion_uart_host),
	.platdata_auto_alloc_size = sizeof(struct sdhci_plat),
};

 static struct sdhci_plat minion_plat;

 U_BOOT_DEVICE(minion_device) = {
   .name = "sdhci",
   .platdata = &minion_plat,
 };
