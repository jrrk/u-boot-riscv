/*
 * drivers/mmc/lowrisc.c
 *
 * SD/MMC driver for Renesas rmobile ARM SoCs.
 *
 * Copyright (C) 2011,2013-2017 Renesas Electronics Corporation
 * Copyright (C) 2014 Nobuhiro Iwamatsu <nobuhiro.iwamatsu.yj@renesas.com>
 * Copyright (C) 2008-2009 Renesas Solutions Corp.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <dm.h>
#include <linux/errno.h>
#include <linux/compat.h>
#include <linux/io.h>
#include <linux/sizes.h>
#include <asm/lowrisc-sdhi.h>
#include <clk.h>

#define CONFIG_SYS_SH_SDHI_NR_CHANNEL 1
#define CONFIG_SH_SDHI_FREQ	97500000
#define DRIVER_NAME "lowrisc_sd"

struct lowrisc_sd_host {
  void __iomem *ioaddr;
  int ch;
  int bus_shift;
  int irq;
  int int_en, width_setting;
  unsigned long quirks;
  unsigned char wait_int;
  unsigned char sd_error;
  unsigned char detect_waiting;
  unsigned char app_cmd;
  void *mmc;
  struct mmc_command {
    int error, opcode, flags, resp[4];
  } *cmd;
  struct mmc_request {
    int app;
    struct mmc_command *cmd;
    struct mmc_data *data;
  } *mrq;
  struct mmc_data *data;
  struct sg_mapping_iter { int length, consumed; void *addr; } sg_miter;
};

#define VERBOSE 0
#define LOGV(l) debug l

void sd_align(struct lowrisc_sd_host *host, int d_align)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[align_reg] = d_align;
}

void sd_clk_div(struct lowrisc_sd_host *host, int clk_div)
{
  volatile uint32_t *sd_base = host->ioaddr;
  /* This section is incomplete */
  sd_base[clk_din_reg] = clk_div;
}

void sd_arg(struct lowrisc_sd_host *host, uint32_t arg)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[arg_reg] = arg;
}

void sd_cmd(struct lowrisc_sd_host *host, uint32_t cmd)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[cmd_reg] = cmd;
}

void sd_setting(struct lowrisc_sd_host *host, int setting)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[setting_reg] = setting;
}

void sd_cmd_start(struct lowrisc_sd_host *host, int sd_cmd)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[start_reg] = sd_cmd;
}

void sd_reset(struct lowrisc_sd_host *host, int sd_rst, int clk_rst, int data_rst, int cmd_rst)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[reset_reg] = ((sd_rst&1) << 3)|((clk_rst&1) << 2)|((data_rst&1) << 1)|((cmd_rst&1) << 0);
}

void sd_blkcnt(struct lowrisc_sd_host *host, int d_blkcnt)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[blkcnt_reg] = d_blkcnt&0xFFFF;
}

void sd_blksize(struct lowrisc_sd_host *host, int d_blksize)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[blksiz_reg] = d_blksize&0xFFF;
}

void sd_timeout(struct lowrisc_sd_host *host, int d_timeout)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[timeout_reg] = d_timeout;
}

void sd_irq_en(struct lowrisc_sd_host *host, int mask)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[irq_en_reg] = mask;
  host->int_en = mask;
}

static void lowrisc_sd_init(struct lowrisc_sd_host *host)
{

}

static void *mmc_priv(struct mmc *mmc)
{
	return (void *)mmc->priv;
}

#if 0
/* Set MMC clock / power */
static void __lowrisc_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	switch (ios->power_mode) {
	case MMC_POWER_OFF:
	  mdelay(1);
	  break;
	case MMC_POWER_UP:
	  break;
	case MMC_POWER_ON:
#if 0
	  mdelay(20);
#endif          
	  break;
	}

	switch (ios->bus_width) {
	case MMC_BUS_WIDTH_1:
	  host->width_setting = 0;
	  break;
	case MMC_BUS_WIDTH_4:
	  host->width_setting = 0x20;
	  break;
	}
}
#endif

static void lowrisc_sd_set_led(struct lowrisc_sd_host *host, unsigned char state)
{
  volatile uint32_t *sd_base = host->ioaddr;
  sd_base[led_reg] = state;
}

static void lowrisc_sd_finish_request(struct lowrisc_sd_host *host)
{
	struct mmc_request *mrq = host->mrq;

	/* Write something to end the command */
	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	sd_reset(host, 0,1,0,1);
	sd_cmd_start(host, 0);
	sd_reset(host, 0,1,1,1);
	lowrisc_sd_set_led(host, 0);
	mmc_request_done(host->mmc, mrq);
}

#ifdef THREADED
static irqreturn_t lowrisc_sd_thread_irq(int irq, void *dev_id)
{
	struct lowrisc_sd_host *host = dev_id;
        volatile uint32_t *sd_base = host->ioaddr;
	struct mmc_data *data = host->data;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	unsigned short *buf;
	int count;
	unsigned long flags;

	LOGV (("lowrisc_sd_thread_irq\n"));

	if (!data) {
		dev_warn(&host->pdev->dev, "Spurious Data IRQ\n");
		if (host->cmd) {
			host->cmd->error = -EIO;
			lowrisc_sd_finish_request(host);
		}
		return IRQ_NONE;
	}
	spin_lock_irqsave(&host->lock, flags);
	if (!sg_miter_next(sg_miter))
		goto done;

	buf = sg_miter->addr;
	/* Ensure we dont read more than one block. The chip will interrupt us
	 * When the next block is available.
	 */
	count = sg_miter->length;
	if (count > data->blocksize)
		count = data->blocksize;

LOG (("count: %08x, flags %08x\n", count,
      data->flags));
	/* Transfer the data */
	if (data->flags & MMC_DATA_READ)
          memcpy(buf, (void*)&sd_base[data_buffer_offset], count >> 2);
	else
          memcpy((void*)&sd_base[data_buffer_offset], buf, count >> 2);

	sg_miter->consumed = count;
	sg_miter_stop(sg_miter);
done:
	spin_unlock_irqrestore(&host->lock, flags);

	return IRQ_HANDLED;
}
#endif
  
static void lowrisc_sd_cmd_irq(struct lowrisc_sd_host *host)
{
	struct mmc_command *cmd = host->cmd;
        volatile uint32_t *sd_base = host->ioaddr;

	LOGV (("lowrisc_sd_cmd_irq\n"));
	
	if (!host->cmd) {
		dev_warn(&host->pdev->dev, "Spurious CMD irq\n");
		return;
	}
	host->cmd = NULL;

        LOGV (("lowrisc_sd_cmd_irq IRQ line %d\n", __LINE__));
	if (cmd->flags & MMC_RSP_PRESENT && cmd->flags & MMC_RSP_136) {
	  int i;
	  LOGV (("lowrisc_sd_cmd_irq IRQ line %d\n", __LINE__));
		/* R2 */
	  for (i = 0;i < 4;i++)
	    {
	    cmd->resp[i] = sd_base[resp0 + (3-i)] << 8;
	    if (i != 3)
	      cmd->resp[i] |= sd_base[resp0 + (2-i)] >> 24;
	    } 
	} else if (cmd->flags & MMC_RSP_PRESENT) {
	  LOGV (("lowrisc_sd_cmd_irq IRQ line %d\n", __LINE__));
		/* R1, R1B, R3, R6, R7 */
	  cmd->resp[0] = sd_base[resp0];
	}

LOGV (("Command IRQ complete %d %d %x\n", cmd->opcode, cmd->error, cmd->flags));

	/* If there is data to handle we will
	 * finish the request in the mmc_data_end_irq handler.*/
	if (host->data)
	  {
	    host->int_en |= SD_CARD_RW_END;
	  }
	else
	  lowrisc_sd_finish_request(host);
}

static void lowrisc_sd_data_end_irq(struct lowrisc_sd_host *host)
{
	struct mmc_data *data = host->data;
        volatile uint32_t *sd_base = host->ioaddr;
	unsigned long flags;
	size_t blksize, len, chunk;
	u32 uninitialized_var(scratch);
	u8 *buf;
	int i = 0;
	
	LOGV (("lowrisc_sd_data_end_irq\n"));

	host->data = NULL;

	if (!data) {
		dev_warn(&host->pdev->dev, "Spurious data end IRQ\n");
		return;
	}

        if (data->flags & MMC_DATA_READ)
	  {
        
	    blksize = data->blocksize;
	    chunk = 0;

	    local_irq_save(flags);

	    while (blksize) {
	      int idx = 0;
	      BUG_ON(!sg_miter_next(&host->sg_miter));
	  
	      len = min(host->sg_miter.length, blksize);
	  
	      blksize -= len;
	      host->sg_miter.consumed = len;
	  
	      buf = host->sg_miter.addr;
	  
	      while (len) {
		if (chunk == 0) {
		  scratch = __be32_to_cpu(sd_base[0x2000 + i++]);
		  chunk = 4;
		}
		
		buf[idx] = scratch & 0xFF;	    
		idx++;
		scratch >>= 8;
		chunk--;
		len--;
	      }
	    }
	    sg_miter_stop(&host->sg_miter);

	    local_irq_restore(flags);
	  }

	LOGV (("Completed data request xfr=%d\n", data->blocks));

        //	iowrite16(0, host->ioaddr + SD_STOPINTERNAL);

	lowrisc_sd_finish_request(host);
}

static irqreturn_t lowrisc_sd_irq(int irq, void *dev_id)
{
	struct lowrisc_sd_host *host = dev_id;
        volatile uint32_t *sd_base = host->ioaddr;
	u32 int_reg, int_status;
	int error = 0, ret = IRQ_HANDLED;

	spin_lock(&host->lock);
	int_status = sd_base[irq_stat_resp];
	int_reg = int_status & host->int_en;

	/* nothing to do: it's not our IRQ */
	if (!int_reg) {
		ret = IRQ_NONE;
		goto irq_end;
	}

	LOGV (("lowrisc_sd IRQ status:%x enabled:%x\n", int_status, host->int_en));

	if (sd_base[wait_resp] >= sd_base[timeout_resp]) {
		error = -ETIMEDOUT;
		LOGV (("lowrisc_sd timeout %d clocks\n", sd_base[timeout_resp]));
	} else if (int_reg & 0) {
		error = -EILSEQ;
		dev_err(&host->pdev->dev, "BadCRC\n");
        }
        
        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

	if (error) {
	  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
		if (host->cmd)
			host->cmd->error = error;

		if (error == -ETIMEDOUT) {
		  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
                  sd_cmd_start(host, 0);
                  sd_setting(host, 0);
		} else {
		  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
			lowrisc_sd_init(host);
                        //			__lowrisc_sd_set_ios(host->mmc, &host->mmc->ios);
			goto irq_end;
		}
	}

        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

        /* Card insert/remove. The mmc controlling code is stateless. */
	if (int_reg & SD_CARD_CARD_REMOVED_0)
	  {
	    int mask = (host->int_en & ~SD_CARD_CARD_REMOVED_0) | SD_CARD_CARD_INSERTED_0;
	    sd_irq_en(host, mask);
	    LOG (("Card removed, mask changed to %d\n", mask));
	    mmc_detect_change(host->mmc, 1);
	  }
	
        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
	if (int_reg & SD_CARD_CARD_INSERTED_0)
	  {
	    int mask = (host->int_en & ~SD_CARD_CARD_INSERTED_0) | SD_CARD_CARD_REMOVED_0 ;
	    sd_irq_en(host, mask);
	    LOG (("Card inserted, mask changed to %d\n", mask));
	    lowrisc_sd_init(host);
	    mmc_detect_change(host->mmc, 1);
	  }

        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
	/* Command completion */
	if (int_reg & SD_CARD_RESP_END) {
	  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

		lowrisc_sd_cmd_irq(host);
		host->int_en &= ~SD_CARD_RESP_END;
	}

        LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));
	/* Data transfer completion */
	if (int_reg & SD_CARD_RW_END) {
	  LOGV (("lowrisc_sd IRQ line %d\n", __LINE__));

		lowrisc_sd_data_end_irq(host);
		host->int_en &= ~SD_CARD_RW_END;
	}
irq_end:
        sd_irq_en(host, host->int_en);
	spin_unlock(&host->lock);
	return ret;
}

static void lowrisc_sd_start_cmd(struct lowrisc_sd_host *host, struct mmc_command *cmd, int arg)
{
  int setting = 0;
  int timeout = 1000000;
  struct mmc_data *data = host->data;
  volatile uint32_t *sd_base = host->ioaddr;
  spin_lock(&host->lock);

  LOGV (("Command opcode: %d\n", cmd->opcode));
/*
  if (cmd->opcode == MMC_STOP_TRANSMISSION) {
    sd_cmd(host, SD_STOPINT_ISSUE_CMD12);

    cmd->resp[0] = cmd->opcode;
    cmd->resp[1] = 0;
    cmd->resp[2] = 0;
    cmd->resp[3] = 0;
    
    lowrisc_sd_finish_request(host);
    return;
  }
*/
  if (!(cmd->flags & MMC_RSP_PRESENT))
    setting = 0;
  else if (cmd->flags & MMC_RSP_136)
    setting = 3;
  else if (cmd->flags & MMC_RSP_BUSY)
    setting = 1;
  else
    setting = 1;
  setting |= host->width_setting;
  
  host->cmd = cmd;
  
  if (cmd->opcode == R1_APP_CMD)
    {
      /* placeholder */
    }
  
  if (cmd->opcode == MMC_CMD_GO_IDLE_STATE)
    {
      /* placeholder */
    }

  LOGV (("testing data flags\n"));
  if (data) {
    setting |= 0x4;
    if (data->flags & MMC_DATA_READ)
      setting |= 0x10;
    else
      {
      setting |= 0x8;
      }
  }

  LOGV (("writing registers\n"));
  /* Send the command */
  sd_reset(host, 0,1,0,1);
  sd_align(host, 0);
  sd_arg(host, arg);
  sd_cmd(host, cmd->opcode);
  sd_setting(host, setting);
  sd_cmd_start(host, 0);
  sd_reset(host, 0,1,1,1);
  sd_timeout(host, timeout);
  /* start the transaction */ 
  sd_cmd_start(host, 1);
  LOGV (("enabling interrupt\n"));
  sd_irq_en(host, sd_base[irq_en_resp] | SD_CARD_RESP_END);
spin_unlock(&host->lock);
 LOGV (("leaving lowrisc_sd_start_cmd\n"));
}

static void lowrisc_sd_start_data(struct lowrisc_sd_host *host, struct mmc_data *data)
{
	unsigned int flags = 0;

	LOGV (("setup data transfer: blocksize %08x  nr_blocks %d, flags: %08x\n",
	      data->blocksize, data->blocks, data->flags));

	host->data = data;

	/* Set transfer length and blocksize */
	sd_blkcnt(host, data->blocks);
	sd_blksize(host, data->blocksize);

        if (!(data->flags & MMC_DATA_READ))
	  {
        volatile uint32_t *sd_base = host->ioaddr;
	struct mmc_data *data = host->data;
	if (sg_miter_next(&host->sg_miter))
{
  size_t blksize, len, chunk;
  u32 scratch, i = 0;
  u8 *buf;
  LOGV (("count: %08x, flags %08x\n", data->blocksize, data->flags));

	blksize = data->blocksize;
	chunk = 0;
	scratch = 0;

	len = min(host->sg_miter.length, blksize);

	blksize -= len;
	host->sg_miter.consumed = len;

	buf = host->sg_miter.addr;

	while (len) {
			scratch |= (u32)*buf << (chunk * 8);

			buf++;
			chunk++;
			len--;

			if ((chunk == 4) || ((len == 0) && (blksize == 0))) {
			  sd_base[0x2000 + i++] = __cpu_to_be32(scratch);
				chunk = 0;
				scratch = 0;
			}
	}

	sg_miter_stop(&host->sg_miter);
	  }
        }
}

/* Process requests from the MMC layer */
static void lowrisc_sd_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint32_t *sd_base = host->ioaddr;
	unsigned long flags;

	/* abort if card not present */
	if (sd_base[detect_resp]) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);

	host->mrq = mrq;

	if (mrq->data)
		lowrisc_sd_start_data(host, mrq->data);

	lowrisc_sd_set_led(host, 1);

	lowrisc_sd_start_cmd(host, mrq->cmd, mrq->app);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void lowrisc_sd_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__lowrisc_sd_set_ios(mmc, ios);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int lowrisc_sd_get_ro(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint32_t *sd_base = host->ioaddr;
	return sd_base[detect_resp];
}

static int lowrisc_sd_get_cd(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint32_t *sd_base = host->ioaddr;

	return !sd_base[detect_resp];
}

static int lowrisc_sd_card_busy(struct mmc_host *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);
        volatile uint32_t *sd_base = host->ioaddr;
	return sd_base[resp0] >> 31;
}

static inline void lowrisc_writeq(struct lowrisc_sd_host *host, int reg, u64 val)
{
  debug("lowrisc_writeq(%p,%x,%x);\n", host, reg, val);
}

static inline u64 lowrisc_readq(struct lowrisc_sd_host *host, int reg)
{
  debug("lowrisc_readq(%p,%x,%x);\n", host, reg);
  return 0;
}

static inline void lowrisc_writew(struct lowrisc_sd_host *host, int reg, u16 val)
{
  debug("lowrisc_writew(%p,%x,%x);\n", host, reg, val);
}

static inline u16 lowrisc_readw(struct lowrisc_sd_host *host, int reg)
{
  debug("lowrisc_readw(%p,%x,%x);\n", host, reg);
  return 0;
}

static void lowrisc_detect(struct lowrisc_sd_host *host)
{
  debug("lowrisc_detect(%p);\n", host);
	lowrisc_writew(host, SDHI_OPTION,
		       OPT_BUS_WIDTH_1 | lowrisc_readw(host, SDHI_OPTION));

	host->detect_waiting = 0;
}

static int lowrisc_intr(void *dev_id)
{
	struct lowrisc_sd_host *host = dev_id;
	int state1 = 0, state2 = 0;

	state1 = lowrisc_readw(host, SDHI_INFO1);
	state2 = lowrisc_readw(host, SDHI_INFO2);

	debug("%s: state1 = %x, state2 = %x\n", __func__, state1, state2);

	/* CARD Insert */
	if (state1 & INFO1_CARD_IN) {
		lowrisc_writew(host, SDHI_INFO1, ~INFO1_CARD_IN);
		if (!host->detect_waiting) {
			host->detect_waiting = 1;
			lowrisc_detect(host);
		}
		lowrisc_writew(host, SDHI_INFO1_MASK, INFO1M_RESP_END |
			       INFO1M_ACCESS_END | INFO1M_CARD_IN |
			       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
		return -EAGAIN;
	}
	/* CARD Removal */
	if (state1 & INFO1_CARD_RE) {
		lowrisc_writew(host, SDHI_INFO1, ~INFO1_CARD_RE);
		if (!host->detect_waiting) {
			host->detect_waiting = 1;
			lowrisc_detect(host);
		}
		lowrisc_writew(host, SDHI_INFO1_MASK, INFO1M_RESP_END |
			       INFO1M_ACCESS_END | INFO1M_CARD_RE |
			       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
		lowrisc_writew(host, SDHI_SDIO_INFO1_MASK, SDIO_INFO1M_ON);
		lowrisc_writew(host, SDHI_SDIO_MODE, SDIO_MODE_OFF);
		return -EAGAIN;
	}

	if (state2 & INFO2_ALL_ERR) {
		lowrisc_writew(host, SDHI_INFO2,
			       (unsigned short)~(INFO2_ALL_ERR));
		lowrisc_writew(host, SDHI_INFO2_MASK,
			       INFO2M_ALL_ERR |
			       lowrisc_readw(host, SDHI_INFO2_MASK));
		host->sd_error = 1;
		host->wait_int = 1;
		return 0;
	}
	/* Respons End */
	if (state1 & INFO1_RESP_END) {
		lowrisc_writew(host, SDHI_INFO1, ~INFO1_RESP_END);
		lowrisc_writew(host, SDHI_INFO1_MASK,
			       INFO1M_RESP_END |
			       lowrisc_readw(host, SDHI_INFO1_MASK));
		host->wait_int = 1;
		return 0;
	}
	/* SD_BUF Read Enable */
	if (state2 & INFO2_BRE_ENABLE) {
		lowrisc_writew(host, SDHI_INFO2, ~INFO2_BRE_ENABLE);
		lowrisc_writew(host, SDHI_INFO2_MASK,
			       INFO2M_BRE_ENABLE | INFO2M_BUF_ILL_READ |
			       lowrisc_readw(host, SDHI_INFO2_MASK));
		host->wait_int = 1;
		return 0;
	}
	/* SD_BUF Write Enable */
	if (state2 & INFO2_BWE_ENABLE) {
		lowrisc_writew(host, SDHI_INFO2, ~INFO2_BWE_ENABLE);
		lowrisc_writew(host, SDHI_INFO2_MASK,
			       INFO2_BWE_ENABLE | INFO2M_BUF_ILL_WRITE |
			       lowrisc_readw(host, SDHI_INFO2_MASK));
		host->wait_int = 1;
		return 0;
	}
	/* Access End */
	if (state1 & INFO1_ACCESS_END) {
		lowrisc_writew(host, SDHI_INFO1, ~INFO1_ACCESS_END);
		lowrisc_writew(host, SDHI_INFO1_MASK,
			       INFO1_ACCESS_END |
			       lowrisc_readw(host, SDHI_INFO1_MASK));
		host->wait_int = 1;
		return 0;
	}
	return -EAGAIN;
}

static int lowrisc_wait_interrupt_flag(struct lowrisc_sd_host *host)
{
	int timeout = 10000000;

	while (1) {
		timeout--;
		if (timeout < 0) {
			debug(DRIVER_NAME": %s timeout\n", __func__);
			return 0;
		}

		if (!lowrisc_intr(host))
			break;

		udelay(1);	/* 1 usec */
	}

	return 1; /* Return value: NOT 0 = complete waiting */
}

static int lowrisc_clock_control(struct lowrisc_sd_host *host, unsigned long clk)
{
	u32 clkdiv, i, timeout;

	if (lowrisc_readw(host, SDHI_INFO2) & (1 << 14)) {
		printf(DRIVER_NAME": Busy state ! Cannot change the clock\n");
		return -EBUSY;
	}

	lowrisc_writew(host, SDHI_CLK_CTRL,
		       ~CLK_ENABLE & lowrisc_readw(host, SDHI_CLK_CTRL));

	if (clk == 0)
		return -EIO;

	clkdiv = 0x80;
	i = CONFIG_SH_SDHI_FREQ >> (0x8 + 1);
	for (; clkdiv && clk >= (i << 1); (clkdiv >>= 1))
		i <<= 1;

	lowrisc_writew(host, SDHI_CLK_CTRL, clkdiv);

	timeout = 100000;
	/* Waiting for SD Bus busy to be cleared */
	while (timeout--) {
		if ((lowrisc_readw(host, SDHI_INFO2) & 0x2000))
			break;
	}

	if (timeout)
		lowrisc_writew(host, SDHI_CLK_CTRL,
			       CLK_ENABLE | lowrisc_readw(host, SDHI_CLK_CTRL));
	else
		return -EBUSY;

	return 0;
}

static int lowrisc_sync_reset(struct lowrisc_sd_host *host)
{
	u32 timeout;
	lowrisc_writew(host, SDHI_SOFT_RST, SOFT_RST_ON);
	lowrisc_writew(host, SDHI_SOFT_RST, SOFT_RST_OFF);
	lowrisc_writew(host, SDHI_CLK_CTRL,
		       CLK_ENABLE | lowrisc_readw(host, SDHI_CLK_CTRL));

	timeout = 100000;
	while (timeout--) {
		if (!(lowrisc_readw(host, SDHI_INFO2) & INFO2_CBUSY))
			break;
		udelay(100);
	}

	if (!timeout)
		return -EBUSY;

	if (host->quirks & SH_SDHI_QUIRK_16BIT_BUF)
		lowrisc_writew(host, SDHI_HOST_MODE, 1);

	return 0;
}

static int lowrisc_error_manage(struct lowrisc_sd_host *host)
{
	unsigned short e_state1, e_state2;
	int ret;

	host->sd_error = 0;
	host->wait_int = 0;

	e_state1 = lowrisc_readw(host, SDHI_ERR_STS1);
	e_state2 = lowrisc_readw(host, SDHI_ERR_STS2);
	if (e_state2 & ERR_STS2_SYS_ERROR) {
		if (e_state2 & ERR_STS2_RES_STOP_TIMEOUT)
			ret = -ETIMEDOUT;
		else
			ret = -EILSEQ;
		debug("%s: ERR_STS2 = %04x\n",
		      DRIVER_NAME, lowrisc_readw(host, SDHI_ERR_STS2));
		lowrisc_sync_reset(host);

		lowrisc_writew(host, SDHI_INFO1_MASK,
			       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
		return ret;
	}
	if (e_state1 & ERR_STS1_CRC_ERROR || e_state1 & ERR_STS1_CMD_ERROR)
		ret = -EILSEQ;
	else
		ret = -ETIMEDOUT;

	debug("%s: ERR_STS1 = %04x\n",
	      DRIVER_NAME, lowrisc_readw(host, SDHI_ERR_STS1));
	lowrisc_sync_reset(host);
	lowrisc_writew(host, SDHI_INFO1_MASK,
		       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
	return ret;
}

static int lowrisc_single_read(struct lowrisc_sd_host *host, struct mmc_data *data)
{
	long time;
	unsigned short blocksize, i;
	unsigned short *p = (unsigned short *)data->dest;
	u64 *q = (u64 *)data->dest;

	if ((unsigned long)p & 0x00000001) {
		debug(DRIVER_NAME": %s: The data pointer is unaligned.",
		      __func__);
		return -EIO;
	}

	host->wait_int = 0;
	lowrisc_writew(host, SDHI_INFO2_MASK,
		       ~(INFO2M_BRE_ENABLE | INFO2M_BUF_ILL_READ) &
		       lowrisc_readw(host, SDHI_INFO2_MASK));
	lowrisc_writew(host, SDHI_INFO1_MASK,
		       ~INFO1M_ACCESS_END &
		       lowrisc_readw(host, SDHI_INFO1_MASK));
	time = lowrisc_wait_interrupt_flag(host);
	if (time == 0 || host->sd_error != 0)
		return lowrisc_error_manage(host);

	host->wait_int = 0;
	blocksize = lowrisc_readw(host, SDHI_SIZE);
	if (host->quirks & SH_SDHI_QUIRK_64BIT_BUF)
		for (i = 0; i < blocksize / 8; i++)
			*q++ = lowrisc_readq(host, SDHI_BUF0);
	else
		for (i = 0; i < blocksize / 2; i++)
			*p++ = lowrisc_readw(host, SDHI_BUF0);

	time = lowrisc_wait_interrupt_flag(host);
	if (time == 0 || host->sd_error != 0)
		return lowrisc_error_manage(host);

	host->wait_int = 0;
	return 0;
}

static int lowrisc_multi_read(struct lowrisc_sd_host *host, struct mmc_data *data)
{
	long time;
	unsigned short blocksize, i, sec;
	unsigned short *p = (unsigned short *)data->dest;
	u64 *q = (u64 *)data->dest;

	if ((unsigned long)p & 0x00000001) {
		debug(DRIVER_NAME": %s: The data pointer is unaligned.",
		      __func__);
		return -EIO;
	}

	debug("%s: blocks = %d, blocksize = %d\n",
	      __func__, data->blocks, data->blocksize);

	host->wait_int = 0;
	for (sec = 0; sec < data->blocks; sec++) {
		lowrisc_writew(host, SDHI_INFO2_MASK,
			       ~(INFO2M_BRE_ENABLE | INFO2M_BUF_ILL_READ) &
			       lowrisc_readw(host, SDHI_INFO2_MASK));

		time = lowrisc_wait_interrupt_flag(host);
		if (time == 0 || host->sd_error != 0)
			return lowrisc_error_manage(host);

		host->wait_int = 0;
		blocksize = lowrisc_readw(host, SDHI_SIZE);
		if (host->quirks & SH_SDHI_QUIRK_64BIT_BUF)
			for (i = 0; i < blocksize / 8; i++)
				*q++ = lowrisc_readq(host, SDHI_BUF0);
		else
			for (i = 0; i < blocksize / 2; i++)
				*p++ = lowrisc_readw(host, SDHI_BUF0);
	}

	return 0;
}

static int lowrisc_single_write(struct lowrisc_sd_host *host,
		struct mmc_data *data)
{
	long time;
	unsigned short blocksize, i;
	const unsigned short *p = (const unsigned short *)data->src;
	const u64 *q = (const u64 *)data->src;

	if ((unsigned long)p & 0x00000001) {
		debug(DRIVER_NAME": %s: The data pointer is unaligned.",
		      __func__);
		return -EIO;
	}

	debug("%s: blocks = %d, blocksize = %d\n",
	      __func__, data->blocks, data->blocksize);

	host->wait_int = 0;
	lowrisc_writew(host, SDHI_INFO2_MASK,
		       ~(INFO2M_BWE_ENABLE | INFO2M_BUF_ILL_WRITE) &
		       lowrisc_readw(host, SDHI_INFO2_MASK));
	lowrisc_writew(host, SDHI_INFO1_MASK,
		       ~INFO1M_ACCESS_END &
		       lowrisc_readw(host, SDHI_INFO1_MASK));

	time = lowrisc_wait_interrupt_flag(host);
	if (time == 0 || host->sd_error != 0)
		return lowrisc_error_manage(host);

	host->wait_int = 0;
	blocksize = lowrisc_readw(host, SDHI_SIZE);
	if (host->quirks & SH_SDHI_QUIRK_64BIT_BUF)
		for (i = 0; i < blocksize / 8; i++)
			lowrisc_writeq(host, SDHI_BUF0, *q++);
	else
		for (i = 0; i < blocksize / 2; i++)
			lowrisc_writew(host, SDHI_BUF0, *p++);

	time = lowrisc_wait_interrupt_flag(host);
	if (time == 0 || host->sd_error != 0)
		return lowrisc_error_manage(host);

	host->wait_int = 0;
	return 0;
}

static int lowrisc_multi_write(struct lowrisc_sd_host *host, struct mmc_data *data)
{
	long time;
	unsigned short i, sec, blocksize;
	const unsigned short *p = (const unsigned short *)data->src;
	const u64 *q = (const u64 *)data->src;

	debug("%s: blocks = %d, blocksize = %d\n",
	      __func__, data->blocks, data->blocksize);

	host->wait_int = 0;
	for (sec = 0; sec < data->blocks; sec++) {
		lowrisc_writew(host, SDHI_INFO2_MASK,
			       ~(INFO2M_BWE_ENABLE | INFO2M_BUF_ILL_WRITE) &
			       lowrisc_readw(host, SDHI_INFO2_MASK));

		time = lowrisc_wait_interrupt_flag(host);
		if (time == 0 || host->sd_error != 0)
			return lowrisc_error_manage(host);

		host->wait_int = 0;
		blocksize = lowrisc_readw(host, SDHI_SIZE);
		if (host->quirks & SH_SDHI_QUIRK_64BIT_BUF)
			for (i = 0; i < blocksize / 8; i++)
				lowrisc_writeq(host, SDHI_BUF0, *q++);
		else
			for (i = 0; i < blocksize / 2; i++)
				lowrisc_writew(host, SDHI_BUF0, *p++);
	}

	return 0;
}

static void lowrisc_get_response(struct lowrisc_sd_host *host, struct mmc_cmd *cmd)
{
	unsigned short i, j, cnt = 1;
	unsigned short resp[8];

	if (cmd->resp_type & MMC_RSP_136) {
		cnt = 4;
		resp[0] = lowrisc_readw(host, SDHI_RSP00);
		resp[1] = lowrisc_readw(host, SDHI_RSP01);
		resp[2] = lowrisc_readw(host, SDHI_RSP02);
		resp[3] = lowrisc_readw(host, SDHI_RSP03);
		resp[4] = lowrisc_readw(host, SDHI_RSP04);
		resp[5] = lowrisc_readw(host, SDHI_RSP05);
		resp[6] = lowrisc_readw(host, SDHI_RSP06);
		resp[7] = lowrisc_readw(host, SDHI_RSP07);

		/* SDHI REGISTER SPECIFICATION */
		for (i = 7, j = 6; i > 0; i--) {
			resp[i] = (resp[i] << 8) & 0xff00;
			resp[i] |= (resp[j--] >> 8) & 0x00ff;
		}
		resp[0] = (resp[0] << 8) & 0xff00;
	} else {
		resp[0] = lowrisc_readw(host, SDHI_RSP00);
		resp[1] = lowrisc_readw(host, SDHI_RSP01);
	}

#if defined(__BIG_ENDIAN_BITFIELD)
	if (cnt == 4) {
		cmd->response[0] = (resp[6] << 16) | resp[7];
		cmd->response[1] = (resp[4] << 16) | resp[5];
		cmd->response[2] = (resp[2] << 16) | resp[3];
		cmd->response[3] = (resp[0] << 16) | resp[1];
	} else {
		cmd->response[0] = (resp[0] << 16) | resp[1];
	}
#else
	if (cnt == 4) {
		cmd->response[0] = (resp[7] << 16) | resp[6];
		cmd->response[1] = (resp[5] << 16) | resp[4];
		cmd->response[2] = (resp[3] << 16) | resp[2];
		cmd->response[3] = (resp[1] << 16) | resp[0];
	} else {
		cmd->response[0] = (resp[1] << 16) | resp[0];
	}
#endif /* __BIG_ENDIAN_BITFIELD */
}

static unsigned short lowrisc_set_cmd(struct lowrisc_sd_host *host,
			struct mmc_data *data, unsigned short opc)
{
	if (host->app_cmd) {
		if (!data)
			host->app_cmd = 0;
		return opc | BIT(6);
	}

	switch (opc) {
	case MMC_CMD_SWITCH:
		return opc | (data ? 0x1c00 : 0x40);
	case MMC_CMD_SEND_EXT_CSD:
		return opc | (data ? 0x1c00 : 0);
	case MMC_CMD_SEND_OP_COND:
		return opc | 0x0700;
	case MMC_CMD_APP_CMD:
		host->app_cmd = 1;
	default:
		return opc;
	}
}

static unsigned short lowrisc_data_trans(struct lowrisc_sd_host *host,
			struct mmc_data *data, unsigned short opc)
{
	if (host->app_cmd) {
		host->app_cmd = 0;
		switch (opc) {
		case SD_CMD_APP_SEND_SCR:
		case SD_CMD_APP_SD_STATUS:
			return lowrisc_single_read(host, data);
		default:
			printf(DRIVER_NAME": SD: NOT SUPPORT APP CMD = d'%04d\n",
				opc);
			return -EINVAL;
		}
	} else {
		switch (opc) {
		case MMC_CMD_WRITE_MULTIPLE_BLOCK:
			return lowrisc_multi_write(host, data);
		case MMC_CMD_READ_MULTIPLE_BLOCK:
			return lowrisc_multi_read(host, data);
		case MMC_CMD_WRITE_SINGLE_BLOCK:
			return lowrisc_single_write(host, data);
		case MMC_CMD_READ_SINGLE_BLOCK:
		case MMC_CMD_SWITCH:
		case MMC_CMD_SEND_EXT_CSD:;
			return lowrisc_single_read(host, data);
		default:
			printf(DRIVER_NAME": SD: NOT SUPPORT CMD = d'%04d\n", opc);
			return -EINVAL;
		}
	}
}

static int lowrisc_start_cmd(struct lowrisc_sd_host *host,
			struct mmc_data *data, struct mmc_cmd *cmd)
{
	long time;
	unsigned short shcmd, opc = cmd->cmdidx;
	int ret = 0;
	unsigned long timeout;

	debug("opc = %d, arg = %x, resp_type = %x\n",
	      opc, cmd->cmdarg, cmd->resp_type);

	if (opc == MMC_CMD_STOP_TRANSMISSION) {
		/* SDHI sends the STOP command automatically by STOP reg */
		lowrisc_writew(host, SDHI_INFO1_MASK, ~INFO1M_ACCESS_END &
			       lowrisc_readw(host, SDHI_INFO1_MASK));

		time = lowrisc_wait_interrupt_flag(host);
		if (time == 0 || host->sd_error != 0)
			return lowrisc_error_manage(host);

		lowrisc_get_response(host, cmd);
		return 0;
	}

	if (data) {
		if ((opc == MMC_CMD_READ_MULTIPLE_BLOCK) ||
		    opc == MMC_CMD_WRITE_MULTIPLE_BLOCK) {
			lowrisc_writew(host, SDHI_STOP, STOP_SEC_ENABLE);
			lowrisc_writew(host, SDHI_SECCNT, data->blocks);
		}
		lowrisc_writew(host, SDHI_SIZE, data->blocksize);
	}

	shcmd = lowrisc_set_cmd(host, data, opc);

	/*
	 *  U-Boot cannot use interrupt.
	 *  So this flag may not be clear by timing
	 */
	lowrisc_writew(host, SDHI_INFO1, ~INFO1_RESP_END);

	lowrisc_writew(host, SDHI_INFO1_MASK,
		       INFO1M_RESP_END | lowrisc_readw(host, SDHI_INFO1_MASK));
	lowrisc_writew(host, SDHI_ARG0,
		       (unsigned short)(cmd->cmdarg & ARG0_MASK));
	lowrisc_writew(host, SDHI_ARG1,
		       (unsigned short)((cmd->cmdarg >> 16) & ARG1_MASK));

	timeout = 100000;
	/* Waiting for SD Bus busy to be cleared */
	while (timeout--) {
		if ((lowrisc_readw(host, SDHI_INFO2) & 0x2000))
			break;
	}

	host->wait_int = 0;
	lowrisc_writew(host, SDHI_INFO1_MASK,
		       ~INFO1M_RESP_END & lowrisc_readw(host, SDHI_INFO1_MASK));
	lowrisc_writew(host, SDHI_INFO2_MASK,
		       ~(INFO2M_CMD_ERROR | INFO2M_CRC_ERROR |
		       INFO2M_END_ERROR | INFO2M_TIMEOUT |
		       INFO2M_RESP_TIMEOUT | INFO2M_ILA) &
		       lowrisc_readw(host, SDHI_INFO2_MASK));

	lowrisc_writew(host, SDHI_CMD, (unsigned short)(shcmd & CMD_MASK));
	time = lowrisc_wait_interrupt_flag(host);
	if (!time) {
		host->app_cmd = 0;
		return lowrisc_error_manage(host);
	}

	if (host->sd_error) {
		switch (cmd->cmdidx) {
		case MMC_CMD_ALL_SEND_CID:
		case MMC_CMD_SELECT_CARD:
		case SD_CMD_SEND_IF_COND:
		case MMC_CMD_APP_CMD:
			ret = -ETIMEDOUT;
			break;
		default:
			debug(DRIVER_NAME": Cmd(d'%d) err\n", opc);
			debug(DRIVER_NAME": cmdidx = %d\n", cmd->cmdidx);
			ret = lowrisc_error_manage(host);
			break;
		}
		host->sd_error = 0;
		host->wait_int = 0;
		host->app_cmd = 0;
		return ret;
	}

	if (lowrisc_readw(host, SDHI_INFO1) & INFO1_RESP_END) {
		host->app_cmd = 0;
		return -EINVAL;
	}

	if (host->wait_int) {
		lowrisc_get_response(host, cmd);
		host->wait_int = 0;
	}

	if (data)
		ret = lowrisc_data_trans(host, data, opc);

	debug("ret = %d, resp = %08x, %08x, %08x, %08x\n",
	      ret, cmd->response[0], cmd->response[1],
	      cmd->response[2], cmd->response[3]);
	return ret;
}

static int lowrisc_send_cmd_common(struct lowrisc_sd_host *host,
				   struct mmc_cmd *cmd, struct mmc_data *data)
{
	host->sd_error = 0;

	return lowrisc_start_cmd(host, data, cmd);
}

static int lowrisc_set_ios_common(struct lowrisc_sd_host *host, struct mmc *mmc)
{
	int ret;

	ret = lowrisc_clock_control(host, mmc->clock);
	if (ret)
		return -EINVAL;

	if (mmc->bus_width == 8)
		lowrisc_writew(host, SDHI_OPTION,
			       OPT_BUS_WIDTH_8 | (~OPT_BUS_WIDTH_M &
			       lowrisc_readw(host, SDHI_OPTION)));
	else if (mmc->bus_width == 4)
		lowrisc_writew(host, SDHI_OPTION,
			       OPT_BUS_WIDTH_4 | (~OPT_BUS_WIDTH_M &
			       lowrisc_readw(host, SDHI_OPTION)));
	else
		lowrisc_writew(host, SDHI_OPTION,
			       OPT_BUS_WIDTH_1 | (~OPT_BUS_WIDTH_M &
			       lowrisc_readw(host, SDHI_OPTION)));

	debug("clock = %d, buswidth = %d\n", mmc->clock, mmc->bus_width);

	return 0;
}

static int lowrisc_initialize_common(struct lowrisc_sd_host *host)
{
	int ret = lowrisc_sync_reset(host);

	lowrisc_writew(host, SDHI_PORTSEL, USE_1PORT);

	lowrisc_writew(host, SDHI_INFO1_MASK, INFO1M_RESP_END |
		       INFO1M_ACCESS_END | INFO1M_CARD_RE |
		       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);

	return ret;
}

static int lowrisc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			    struct mmc_data *data)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);

	return lowrisc_send_cmd_common(host, cmd, data);
}

static int lowrisc_set_ios(struct mmc *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);

	return lowrisc_set_ios_common(host, mmc);
}

static int lowrisc_initialize(struct mmc *mmc)
{
	struct lowrisc_sd_host *host = mmc_priv(mmc);

	return lowrisc_initialize_common(host);
}

static const struct mmc_ops lowrisc_ops = {
	.send_cmd       = lowrisc_send_cmd,
	.set_ios        = lowrisc_set_ios,
	.init           = lowrisc_initialize,
};

static struct mmc_config lowrisc_cfg = {
	.name           = DRIVER_NAME,
	.ops            = &lowrisc_ops,
	.f_min          = CLKDEV_INIT,
	.f_max          = CLKDEV_HS_DATA,
	.voltages       = MMC_VDD_32_33 | MMC_VDD_33_34,
	.host_caps      = MMC_MODE_4BIT | MMC_MODE_HS,
	.part_type      = PART_TYPE_DOS,
	.b_max          = CONFIG_SYS_MMC_MAX_BLK_COUNT,
};

int lowrisc_init(unsigned long addr, int ch, unsigned long quirks)
{
	int ret = 0;
	struct mmc *mmc;
	struct lowrisc_sd_host *host = NULL;

	if (ch >= CONFIG_SYS_SH_SDHI_NR_CHANNEL)
		return -ENODEV;

	host = malloc(sizeof(struct lowrisc_sd_host));
	if (!host)
		return -ENOMEM;

	mmc = mmc_create(&lowrisc_cfg, host);
	if (!mmc) {
		ret = -1;
		goto error;
	}

	host->ch = ch;
	host->ioaddr = (void __iomem *)addr;
	host->quirks = quirks;

	if (host->quirks & SH_SDHI_QUIRK_64BIT_BUF)
		host->bus_shift = 2;
	else if (host->quirks & SH_SDHI_QUIRK_16BIT_BUF)
		host->bus_shift = 1;

	return ret;
error:
	if (host)
		free(host);
	return ret;
}
