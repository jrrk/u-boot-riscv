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

#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <malloc.h>
#include <errno.h>
#include <mmc.h>
#include "minion_lib.h"

/*
 * Controller registers
 */

#define MINION_UART_DMA_ADDRESS	0x00

#define MINION_UART_BLOCK_SIZE	0x04

#define MINION_UART_BLOCK_COUNT	0x06

#define MINION_UART_ARGUMENT		0x08

#define MINION_UART_TRANSFER_MODE	0x0C
#define  MINION_UART_TRNS_DMA		0x01
#define  MINION_UART_TRNS_BLK_CNT_EN	0x02
#define  MINION_UART_TRNS_ACMD12	0x04
#define  MINION_UART_TRNS_READ	0x10
#define  MINION_UART_TRNS_MULTI	0x20

#define MINION_UART_COMMAND		0x0E
#define  MINION_UART_CMD_RESP_MASK	0x03
#define  MINION_UART_CMD_CRC		0x08
#define  MINION_UART_CMD_INDEX	0x10
#define  MINION_UART_CMD_DATA		0x20
#define  MINION_UART_CMD_ABORTCMD	0xC0

#define  MINION_UART_CMD_RESP_NONE	0x00
#define  MINION_UART_CMD_RESP_LONG	0x01
#define  MINION_UART_CMD_RESP_SHORT	0x02
#define  MINION_UART_CMD_RESP_SHORT_BUSY 0x03

#define MINION_UART_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))
#define MINION_UART_GET_CMD(c) ((c>>8) & 0x3f)

#define MINION_UART_RESPONSE		0x10

#define MINION_UART_BUFFER		0x20

#define MINION_UART_PRESENT_STATE	0x24
#define  MINION_UART_CMD_INHIBIT	0x00000001
#define  MINION_UART_DATA_INHIBIT	0x00000002
#define  MINION_UART_DOING_WRITE	0x00000100
#define  MINION_UART_DOING_READ	0x00000200
#define  MINION_UART_SPACE_AVAILABLE	0x00000400
#define  MINION_UART_DATA_AVAILABLE	0x00000800
#define  MINION_UART_CARD_PRESENT	0x00010000
#define  MINION_UART_CARD_STATE_STABLE	0x00020000
#define  MINION_UART_CARD_DETECT_PIN_LEVEL	0x00040000
#define  MINION_UART_WRITE_PROTECT	0x00080000

#define MINION_UART_HOST_CONTROL	0x28
#define  MINION_UART_POWER_ON		0x01
#define  MINION_UART_CTRL_LED		0x01
#define  MINION_UART_CTRL_4BITBUS	0x02
#define  MINION_UART_CTRL_HISPD	0x04
#define  MINION_UART_CTRL_DMA_MASK	0x18
#define   MINION_UART_CTRL_SDMA	0x00
#define   MINION_UART_CTRL_ADMA1	0x08
#define   MINION_UART_CTRL_ADMA32	0x10
#define   MINION_UART_CTRL_ADMA64	0x18
#define  MINION_UART_CTRL_8BITBUS	0x20
#define  MINION_UART_CTRL_CD_TEST_INS	0x40
#define  MINION_UART_CTRL_CD_TEST	0x80

#define MINION_UART_POWER_CONTROL	0x29
#define  MINION_UART_POWER_180	0x0A
#define  MINION_UART_POWER_300	0x0C
#define  MINION_UART_POWER_330	0x0E

#define MINION_UART_BLOCK_GAP_CONTROL	0x2A
#define MINION_UART_WAKE_UP_CONTROL	0x2B
#define MINION_UART_TIMEOUT_CONTROL	0x2E
#define MINION_UART_SOFTWARE_RESET	0x2F

#define  MINION_UART_WAKE_ON_INT	0x01
#define  MINION_UART_WAKE_ON_INSERT	0x02
#define  MINION_UART_WAKE_ON_REMOVE	0x04

#define MINION_UART_CLOCK_CONTROL	0x2C
#define  MINION_UART_DIVIDER_SHIFT	8
#define  MINION_UART_DIVIDER_HI_SHIFT	6
#define  MINION_UART_DIV_MASK	0xFF
#define  MINION_UART_DIV_MASK_LEN	8
#define  MINION_UART_DIV_HI_MASK	0x300
#define  MINION_UART_PROG_CLOCK_MODE  0x0020
#define  MINION_UART_CLOCK_CARD_EN	0x0004
#define  MINION_UART_CLOCK_INT_STABLE	0x0002
#define  MINION_UART_CLOCK_INT_EN	0x0001

#define  MINION_UART_RESET_ALL	0x01
#define  MINION_UART_RESET_CMD	0x02
#define  MINION_UART_RESET_DATA	0x04

#define MINION_UART_INT_STATUS	0x30
#define MINION_UART_INT_ENABLE	0x34
#define MINION_UART_SIGNAL_ENABLE	0x38
#define  MINION_UART_INT_RESPONSE	0x00000001
#define  MINION_UART_INT_DATA_END	0x00000002
#define  MINION_UART_INT_DMA_END	0x00000008
#define  MINION_UART_INT_SPACE_AVAIL	0x00000010
#define  MINION_UART_INT_DATA_AVAIL	0x00000020
#define  MINION_UART_INT_CARD_INSERT	0x00000040
#define  MINION_UART_INT_CARD_REMOVE	0x00000080
#define  MINION_UART_INT_CARD_INT	0x00000100
#define  MINION_UART_INT_ERROR	0x00008000
#define  MINION_UART_INT_TIMEOUT	0x00010000
#define  MINION_UART_INT_CRC		0x00020000
#define  MINION_UART_INT_END_BIT	0x00040000
#define  MINION_UART_INT_INDEX	0x00080000
#define  MINION_UART_INT_DATA_TIMEOUT	0x00100000
#define  MINION_UART_INT_DATA_CRC	0x00200000
#define  MINION_UART_INT_DATA_END_BIT	0x00400000
#define  MINION_UART_INT_BUS_POWER	0x00800000
#define  MINION_UART_INT_ACMD12ERR	0x01000000
#define  MINION_UART_INT_ADMA_ERROR	0x02000000

#define  MINION_UART_INT_NORMAL_MASK	0x00007FFF
#define  MINION_UART_INT_ERROR_MASK	0xFFFF8000

#define  MINION_UART_INT_CMD_MASK	(MINION_UART_INT_RESPONSE | MINION_UART_INT_TIMEOUT | \
		MINION_UART_INT_CRC | MINION_UART_INT_END_BIT | MINION_UART_INT_INDEX)
#define  MINION_UART_INT_DATA_MASK	(MINION_UART_INT_DATA_END | MINION_UART_INT_DMA_END | \
		MINION_UART_INT_DATA_AVAIL | MINION_UART_INT_SPACE_AVAIL | \
		MINION_UART_INT_DATA_TIMEOUT | MINION_UART_INT_DATA_CRC | \
		MINION_UART_INT_DATA_END_BIT | MINION_UART_INT_ADMA_ERROR)
#define MINION_UART_INT_ALL_MASK	((unsigned int)-1)

#define MINION_UART_ACMD12_ERR	0x3C

/* 3E-3F reserved */

#define MINION_UART_CAPABILITIES	0x40
#define  MINION_UART_TIMEOUT_CLK_MASK	0x0000003F
#define  MINION_UART_TIMEOUT_CLK_SHIFT 0
#define  MINION_UART_TIMEOUT_CLK_UNIT	0x00000080
#define  MINION_UART_CLOCK_BASE_MASK	0x00003F00
#define  MINION_UART_CLOCK_V3_BASE_MASK	0x0000FF00
#define  MINION_UART_CLOCK_BASE_SHIFT	8
#define  MINION_UART_MAX_BLOCK_MASK	0x00030000
#define  MINION_UART_MAX_BLOCK_SHIFT  16
#define  MINION_UART_CAN_DO_8BIT	0x00040000
#define  MINION_UART_CAN_DO_ADMA2	0x00080000
#define  MINION_UART_CAN_DO_ADMA1	0x00100000
#define  MINION_UART_CAN_DO_HISPD	0x00200000
#define  MINION_UART_CAN_DO_SDMA	0x00400000
#define  MINION_UART_CAN_VDD_330	0x01000000
#define  MINION_UART_CAN_VDD_300	0x02000000
#define  MINION_UART_CAN_VDD_180	0x04000000
#define  MINION_UART_CAN_64BIT	0x10000000

#define MINION_UART_CAPABILITIES_1	0x44
#define  MINION_UART_CLOCK_MUL_MASK	0x00FF0000
#define  MINION_UART_CLOCK_MUL_SHIFT	16

#define MINION_UART_MAX_CURRENT	0x48

/* 4C-4F reserved for more max current */

#define MINION_UART_SET_ACMD12_ERROR	0x50
#define MINION_UART_SET_INT_ERROR	0x52

#define MINION_UART_ADMA_ERROR	0x54

/* 55-57 reserved */

#define MINION_UART_ADMA_ADDRESS	0x58

/* 60-FB reserved */

#define MINION_UART_SLOT_INT_STATUS	0xFC

#define MINION_UART_HOST_VERSION	0xFE
#define  MINION_UART_VENDOR_VER_MASK	0xFF00
#define  MINION_UART_VENDOR_VER_SHIFT	8
#define  MINION_UART_SPEC_VER_MASK	0x00FF
#define  MINION_UART_SPEC_VER_SHIFT	0
#define   MINION_UART_SPEC_100	0
#define   MINION_UART_SPEC_200	1
#define   MINION_UART_SPEC_300	2

#define MINION_UART_GET_VERSION(x) (x->version & MINION_UART_SPEC_VER_MASK)

/*
 * End of controller registers.
 */

#define MINION_UART_MAX_DIV_SPEC_200	256
#define MINION_UART_MAX_DIV_SPEC_300	2046

/* to make gcc happy */
struct minion_uart_host;

/*
 * Host SDMA buffer boundary. Valid values from 4K to 512K in powers of 2.
 */
#define MINION_UART_DEFAULT_BOUNDARY_SIZE	(512 * 1024)
#define MINION_UART_DEFAULT_BOUNDARY_ARG	(7)
struct minion_uart_ops {
#ifdef CONFIG_MMC_MINION_UART_IO_ACCESSORS
	u32             (*read_l)(struct minion_uart_host *host, int reg);
	u16             (*read_w)(struct minion_uart_host *host, int reg);
	u8              (*read_b)(struct minion_uart_host *host, int reg);
	void            (*write_l)(struct minion_uart_host *host, u32 val, int reg);
	void            (*write_w)(struct minion_uart_host *host, u16 val, int reg);
	void            (*write_b)(struct minion_uart_host *host, u8 val, int reg);
#endif
};

struct minion_uart_host {
	const char *name;
	unsigned int quirks;
	unsigned int host_caps;
	unsigned int version;
	unsigned int clk_mul;   /* Clock Multiplier value */
	unsigned int clock;
	struct mmc *mmc;
	const struct minion_uart_ops *ops;
	int index;
	int bus_width;
	void (*set_control_reg)(struct minion_uart_host *host);
	void (*set_clock)(int dev_index, unsigned int div);
	uint	voltages;
	struct mmc_config cfg;
        const char *start_addr;
};

static int minion_uart_host_control;
static int minion_uart_power_on;
static int minion_uart_ctrl_4bitbus;
static int minion_uart_ctrl_dma;
static int minion_uart_argument;
static int minion_uart_ctrl_adma32;
static int minion_uart_ctrl_8bitbus;
static int minion_uart_ctrl_cd;
static int minion_uart_ctrl_cd;
static int minion_uart_power_control;
static int minion_uart_power_180;
static int minion_uart_block_gap;
static int minion_uart_wake_up;
static int minion_uart_timeout_control;
static int minion_uart_software_reset;
static int minion_uart_clock_control;
static int minion_uart_int_status;
static int minion_uart_int_enable;
static int minion_uart_signal_enable;
static int minion_uart_present_state;
static int minion_uart_max_current;
static int minion_uart_set_acmd12;
static int minion_uart_set_int;
static int minion_uart_slot_int;
static int minion_uart_host_version;
static int minion_uart_block_count;
static int minion_uart_transfer_mode;
static int minion_uart_block_size;
static int minion_uart_command;
static unsigned setting, blksiz, blkcnt, response[8];

static void minion_uart_write(struct minion_uart_host *host, u32 val, int reg)
{  
  switch (reg)
    {
    case MINION_UART_BLOCK_COUNT	: minion_uart_block_count = val; break;
    case MINION_UART_BLOCK_SIZE	        : minion_uart_block_size = val; break;
    case MINION_UART_HOST_CONTROL	: minion_uart_host_control = val; break;
    case MINION_UART_POWER_ON	        : minion_uart_power_on = val; break;
    case MINION_UART_CTRL_4BITBUS	: minion_uart_ctrl_4bitbus = val; break;
    case MINION_UART_CTRL_DMA_MASK	: minion_uart_ctrl_dma = val; break;
    case MINION_UART_ARGUMENT	        : minion_uart_argument = val; break;
    case MINION_UART_TRANSFER_MODE	: minion_uart_transfer_mode = val; break;

    case MINION_UART_CTRL_ADMA32	: minion_uart_ctrl_adma32 = val; break;
    case MINION_UART_CTRL_8BITBUS	: minion_uart_ctrl_8bitbus = val; break;
    case MINION_UART_CTRL_CD_TEST_INS   : minion_uart_ctrl_cd = val; break;
    case MINION_UART_CTRL_CD_TEST	: minion_uart_ctrl_cd = val; break;
    case MINION_UART_POWER_CONTROL	: minion_uart_power_control = val; break;
    case MINION_UART_POWER_180	        : minion_uart_power_180 = val; break;
    case MINION_UART_COMMAND	        :
      minion_uart_command = val >> 8;
      switch(val & MINION_UART_CMD_RESP_MASK)
	{
	case MINION_UART_CMD_RESP_NONE: setting = 0; break;
	case MINION_UART_CMD_RESP_SHORT: setting = 1; break;
	case MINION_UART_CMD_RESP_SHORT_BUSY: setting = 1; break;
	case MINION_UART_CMD_RESP_LONG: setting = 3; break;
	}
      if (minion_uart_transfer_mode & MINION_UART_TRNS_READ)
	{
	  setting |= 0x14;
	  blkcnt = minion_uart_block_count;
	  blksiz = 512; // should be minion_uart_block_size;
	}
      sd_transaction(minion_uart_command, minion_uart_argument, setting, response);
      log_printf("sd_transaction(%d,0x%.8X,0x%X,resp);\n", minion_uart_command, minion_uart_argument, setting);
      minion_uart_int_status = MINION_UART_INT_RESPONSE;
      break;
    case MINION_UART_BLOCK_GAP_CONTROL	: minion_uart_block_gap = val; break;
    case MINION_UART_WAKE_UP_CONTROL	: minion_uart_wake_up = val; break;
    case MINION_UART_TIMEOUT_CONTROL	:
      uart_write(sd_base+9, val);
      minion_uart_timeout_control = val; 
      break;
    case MINION_UART_SOFTWARE_RESET	:
      minion_uart_software_reset = val;
      minion_uart_timeout_control = 1000; 
      minion_uart_transfer_mode = 0;
      open_handle();
      break;
    case MINION_UART_CLOCK_CONTROL	: minion_uart_clock_control = val; break;
    case MINION_UART_INT_STATUS	: minion_uart_int_status = val; break;
    case MINION_UART_INT_ENABLE	: minion_uart_int_enable = val; break;
    case MINION_UART_SIGNAL_ENABLE	: minion_uart_signal_enable = val; break;
    case MINION_UART_PRESENT_STATE	: minion_uart_present_state = val; break;
    case MINION_UART_MAX_CURRENT	: minion_uart_max_current = val; break;
    case MINION_UART_SET_ACMD12_ERROR	: minion_uart_set_acmd12 = val; break;
    case MINION_UART_SET_INT_ERROR	: minion_uart_set_int = val; break;
    case MINION_UART_SLOT_INT_STATUS	: minion_uart_slot_int = val; break;
    case MINION_UART_HOST_VERSION	: minion_uart_host_version = val; break;
    default: printf("unknown(%d)", reg);
    }
}

static u32 minion_uart_read(struct minion_uart_host *host, int reg)
{
  int cnt;
  unsigned cmd, arg, len;
  switch (reg)
    {
    case MINION_UART_RESPONSE          : return response[0];
    case MINION_UART_RESPONSE+4        : return response[1];
    case MINION_UART_RESPONSE+8        : return response[2];
    case MINION_UART_RESPONSE+12       : return response[3];
    case MINION_UART_INT_STATUS	:
      /*
      cnt = fionread(&cmd, &arg, &len, response);
      if (cnt >= 11)
	{
	  assert(cmd==minion_uart_command);
	  assert(arg==minion_uart_argument);
	  assert(len==setting);
      */
	  return response[4] < minion_uart_timeout_control ? MINION_UART_INT_RESPONSE|MINION_UART_INT_DATA_AVAIL : MINION_UART_INT_ERROR;
      /*
	}
      return 0;
      */
    case MINION_UART_INT_ENABLE	: return minion_uart_int_enable;
    case MINION_UART_PRESENT_STATE	: return MINION_UART_DATA_AVAILABLE;
    case MINION_UART_HOST_VERSION	: return minion_uart_host_version;
    case MINION_UART_CAPABILITIES      : return MINION_UART_CAN_VDD_330;
    case MINION_UART_SOFTWARE_RESET : return 0;
    case MINION_UART_HOST_CONTROL: return minion_uart_host_control;
    case MINION_UART_CLOCK_CONTROL: return minion_uart_clock_control|MINION_UART_CLOCK_INT_STABLE;
    case MINION_UART_BUFFER : return 0;
    default: printf("unknown(%d)", reg);
    }
  return 0;
}

static void minion_uart_reset(struct minion_uart_host *host, u8 mask)
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

static void minion_uart_cmd_done(struct minion_uart_host *host, struct mmc_cmd *cmd)
{
	int i;
	if (cmd->resp_type & MMC_RSP_136) {
		/* CRC is stripped so we need to do some shifting. */
		for (i = 0; i < 4; i++) {
			cmd->response[i] = minion_uart_read(host,
					MINION_UART_RESPONSE + (3-i)*4) << 8;
			if (i != 3)
				cmd->response[i] |= minion_uart_read(host,
						MINION_UART_RESPONSE + (2-i)*4) >> 24;
		}
	} else {
		cmd->response[0] = minion_uart_read(host, MINION_UART_RESPONSE);
	}
}

/*
 * No command will be sent by driver if card is busy, so driver must wait
 * for card ready state.
 * Every time when card is busy after timeout then (last) timeout value will be
 * increased twice but only if it doesn't exceed global defined maximum.
 * Each function call will use last timeout value.
 */
#define MINION_UART_CMD_MAX_TIMEOUT			3200
#define MINION_UART_CMD_DEFAULT_TIMEOUT		100
#define MINION_UART_READ_STATUS_TIMEOUT		1000

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
	u32 mask, flags, mode;
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
		minion_uart_write(host, 0xe, MINION_UART_TIMEOUT_CONTROL);
		mode = MINION_UART_TRNS_BLK_CNT_EN;
		if (data->blocks > 1)
			mode |= MINION_UART_TRNS_MULTI;

		if (data->flags == MMC_DATA_READ)
			mode |= MINION_UART_TRNS_READ;

		if (data->flags == MMC_DATA_READ)
			host->start_addr = data->dest;
		else
			host->start_addr = data->src;

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
		minion_uart_cmd_done(host, cmd);
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
	unsigned int div, clk = 0, timeout, reg;

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

	if (clock == 0)
		return 0;

	if (MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300) {
		/*
		 * Check if the Host Controller supports Programmable Clock
		 * Mode.
		 */
		if (host->clk_mul) {
			for (div = 1; div <= 1024; div++) {
				if ((mmc->cfg->f_max * host->clk_mul / div)
					<= clock)
					break;
			}

			/*
			 * Set Programmable Clock Mode in the Clock
			 * Control register.
			 */
			clk = MINION_UART_PROG_CLOCK_MODE;
			div--;
		} else {
			/* Version 3.00 divisors must be a multiple of 2. */
			if (mmc->cfg->f_max <= clock) {
				div = 1;
			} else {
				for (div = 2;
				     div < MINION_UART_MAX_DIV_SPEC_300;
				     div += 2) {
					if ((mmc->cfg->f_max / div) <= clock)
						break;
				}
			}
			div >>= 1;
		}
	} else {
		/* Version 2.00 divisors must be a power of 2. */
		for (div = 1; div < MINION_UART_MAX_DIV_SPEC_200; div *= 2) {
			if ((mmc->cfg->f_max / div) <= clock)
				break;
		}
		div >>= 1;
	}

	if (host->set_clock)
		host->set_clock(host->index, div);

	clk |= (div & MINION_UART_DIV_MASK) << MINION_UART_DIVIDER_SHIFT;
	clk |= ((div & MINION_UART_DIV_HI_MASK) >> MINION_UART_DIV_MASK_LEN)
		<< MINION_UART_DIVIDER_HI_SHIFT;
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

	if (host->set_control_reg)
		host->set_control_reg(host);

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

int minion_uart_setup_cfg(struct mmc_config *cfg, struct minion_uart_host *host,
		u32 max_clk, u32 min_clk)
{
	u32 caps, caps_1;

	caps = minion_uart_read(host, MINION_UART_CAPABILITIES);

	host->version = minion_uart_read(host, MINION_UART_HOST_VERSION);

	cfg->name = host->name;
#ifndef CONFIG_DM_MMC_OPS
	cfg->ops = &minion_uart_ops;
#endif
	if (max_clk)
		cfg->f_max = max_clk;
	else {
		if (MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300)
			cfg->f_max = (caps & MINION_UART_CLOCK_V3_BASE_MASK) >>
				MINION_UART_CLOCK_BASE_SHIFT;
		else
			cfg->f_max = (caps & MINION_UART_CLOCK_BASE_MASK) >>
				MINION_UART_CLOCK_BASE_SHIFT;
		cfg->f_max *= 1000000;
	}
	if (cfg->f_max == 0) {
		printf("%s: Hardware doesn't specify base clock frequency\n",
		       __func__);
		return -EINVAL;
	}
	if (min_clk)
		cfg->f_min = min_clk;
	else {
		if (MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300)
			cfg->f_min = cfg->f_max / MINION_UART_MAX_DIV_SPEC_300;
		else
			cfg->f_min = cfg->f_max / MINION_UART_MAX_DIV_SPEC_200;
	}
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

	/*
	 * In case of Host Controller v3.00, find out whether clock
	 * multiplier is supported.
	 */
	if (MINION_UART_GET_VERSION(host) >= MINION_UART_SPEC_300) {
		caps_1 = minion_uart_read(host, MINION_UART_CAPABILITIES_1);
		host->clk_mul = (caps_1 & MINION_UART_CLOCK_MUL_MASK) >>
				MINION_UART_CLOCK_MUL_SHIFT;
	}

	return 0;
}

#ifndef CONFIG_MINION_UART_MIN_FREQ
# define CONFIG_MINION_UART_MIN_FREQ	0
#endif

#ifndef CONFIG_MINION_UART_MAX_FREQ
# define CONFIG_MINION_UART_MAX_FREQ	50000000
#endif

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

	ret = minion_uart_setup_cfg(&plat->cfg, host, CONFIG_MINION_UART_MAX_FREQ,
			      CONFIG_MINION_UART_MIN_FREQ);
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
