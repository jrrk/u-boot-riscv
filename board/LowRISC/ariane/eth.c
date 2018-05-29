/*
 * Dummy eth driver based on SMSC LAN9[12]1[567] Network driver
 *
 * (c) 2007 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <net.h>
#include <miiphy.h>

#include "eth.h"

static u64 eth_read(struct eth_device *dev, u32 offset)
{
  u64 ret;
  volatile u64 *ptr = (volatile u64 *)(dev->iobase + offset);
  
  //  debug("eth_read(%llX)\n", ptr);
  ret = *ptr;
  //  debug("eth read => %lX\n", ret);
  return ret;
}

static void eth_write(struct eth_device *dev, u32 offset, u64 val)
{
  volatile u64 *ptr = (volatile u64 *)(dev->iobase + offset);
  //  debug("eth_write(%llX, %lx)\n", ptr, val);
  *ptr = val;
  //  debug("eth written\n");
}

static int lowrisc_detect_chip(struct eth_device *dev)
{
	dev->priv = (void *)chip_ids;

	return 0;
}

static void lowrisc_reset(struct eth_device *dev)
{
	/* Disable interrupts */
  //	lowrisc_reg_write(dev, INT_EN, 0);

  //	lowrisc_reg_write(dev, HW_CFG, HW_CFG_SRST);

	/* Set to LED outputs */
  //	lowrisc_reg_write(dev, GPIO_CFG, 0x70070000);
}

static void lowrisc_handle_mac_address(struct eth_device *dev)
{
	unsigned long addrh, addrl;
	uchar *m = dev->enetaddr;

	addrl = m[5] | (m[4] << 8) | (m[3] << 16) | (m[2] << 24);
	addrh = m[1] | (m[0] << 8);

        eth_write(dev, MACLO_OFFSET, addrl);
        eth_write(dev, MACHI_OFFSET, addrh /* |MACHI_ALLPKTS_MASK */);

	printf(DRIVERNAME ": MAC %pM\n", m);
}

static int lowrisc_eth_phy_read(struct eth_device *dev,
				u8 phy, u8 reg, u16 *val)
{
  *val = 0;

  return 0;
}

static int lowrisc_eth_phy_write(struct eth_device *dev,
				u8 phy, u8 reg, u16  val)
{
  return 0;
}

static int lowrisc_phy_reset(struct eth_device *dev)
{
  return 0;
}

static void lowrisc_phy_configure(struct eth_device *dev)
{  
  printf(DRIVERNAME ": phy initialized\n");
}

static void lowrisc_enable(struct eth_device *dev)
{
	lowrisc_handle_mac_address(dev);
}

static int lowrisc_init(struct eth_device *dev, bd_t * bd)
{
	struct chip_id *id = dev->priv;

	printf(DRIVERNAME ": detected %s controller\n", id->name);

	return 0;
}

static int lowrisc_send(struct eth_device *dev, void *packet, int len)
{
	u64 *alloc = (u64*)packet;
	u32 i;
	u32 rslt;
        u8 *ptr = (u8*)packet;

        rslt = eth_read(dev, TPLR_OFFSET);
        if (rslt & TPLR_BUSY_MASK)
          debug("TX Busy Status = %x, len = %d, ignoring\n", rslt, len);
        for (i = 0; i < (((len-1)|7)+1)/8; i++)
          {
            eth_write(dev, TXBUFF_OFFSET+(i<<3), alloc[i]);
          }
        eth_write(dev, TPLR_OFFSET, len);
        //        debug("lowrisc_tx: len=%d\n", len);
        return 0;
}

static void lowrisc_halt(struct eth_device *dev)
{
	lowrisc_reset(dev);
	lowrisc_handle_mac_address(dev);
}

static int lowrisc_rx(struct eth_device *dev)
{
	u64 *alloc = (u64 *)net_rx_packets[0];
	u32 pktlen = 0;

        if (eth_read(dev, RSR_OFFSET) & RSR_RECV_DONE_MASK)
          {
            int fcs = eth_read(dev, RFCS_OFFSET);
            int rplr = eth_read(dev, RPLR_OFFSET);
            //            debug("lowrisc_rx: fcs=%x\n", fcs);
            pktlen = (rplr & RPLR_LENGTH_MASK) - 4; /* discard FCS bytes */
            if ((pktlen >= 14) && (fcs == 0xc704dd7b) && (pktlen <= 1536))
              {
                int i, rnd = (((pktlen-1)|7)+1)/8; /* round to a multiple of 8 */
                for (i = 0; i < rnd; i++)
                      {
                        alloc[i] = eth_read(dev, RXBUFF_OFFSET+(i<<3));
                      }
              }
            else
              pktlen = 0;
            /* acknowledge, even if an error occurs, to reset irq */
            eth_write(dev, RSR_OFFSET, 0);
          }
        if (pktlen)
          {
#if 0
            u8 *ptr = alloc;
            int i;
            for (i = 0; i < pktlen; i++)
              {
                if (i%16 == 0) debug("\n%.4X: ", i);
                debug("%.02X ", ptr[i]);
              }
            debug("\n");
#endif            
          net_process_received_packet(net_rx_packets[0], pktlen);
          }
	return 0;
}

#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)
/* wrapper for lowrisc_eth_phy_read */
static int lowrisc_miiphy_read(struct mii_dev *bus, int phy, int devad,
			       int reg)
{
	u16 val = 0;
	struct eth_device *dev = eth_get_dev_by_name(bus->name);
	if (dev) {
		int retval = lowrisc_eth_phy_read(dev, phy, reg, &val);
		if (retval < 0)
			return retval;
		return val;
	}
	return -ENODEV;
}
/* wrapper for lowrisc_eth_phy_write */
static int lowrisc_miiphy_write(struct mii_dev *bus, int phy, int devad,
				int reg, u16 val)
{
	struct eth_device *dev = eth_get_dev_by_name(bus->name);
	if (dev)
		return lowrisc_eth_phy_write(dev, phy, reg, val);
	return -ENODEV;
}
#endif

int lowrisc_initialize(u8 dev_num, int base_addr)
{
  unsigned long addrl, addrh;
  struct eth_device *dev;
  
  dev = malloc(sizeof(*dev));
  if (!dev) {
    return -1;
  }
  memset(dev, 0, sizeof(*dev));
  
  dev->iobase = base_addr;
  
  debug("dev->iobase = %lx\n", dev->iobase);
        
  /* Try to detect chip. Will fail if not present. */
  if (lowrisc_detect_chip(dev)) {
    free(dev);
    return 0;
  }

  addrh = eth_read(dev, MACHI_OFFSET)&MACHI_MACADDR_MASK;
  addrl = eth_read(dev, MACLO_OFFSET);

  debug("MAC addr = %lX:%lX\n", addrh, addrl);
  if (!(addrl == 0xffffffff && addrh == 0x0000ffff)) {
    /* address is obtained from optional eeprom */
    dev->enetaddr[5] = addrl;
    dev->enetaddr[4] = addrl >>  8;
    dev->enetaddr[3] = addrl >> 16;
    dev->enetaddr[2] = addrl >> 24;
    dev->enetaddr[1] = addrh;
    dev->enetaddr[0] = addrh >> 8;
  }
  
  dev->init = lowrisc_init;
  dev->halt = lowrisc_halt;
  dev->send = lowrisc_send;
  dev->recv = lowrisc_rx;
  sprintf(dev->name, "%s-%hu", DRIVERNAME, dev_num);

  debug("calling eth_register\n");
  eth_register(dev);

#if defined(CONFIG_MII) || defined(CONFIG_CMD_MII)
  int retval;
  struct mii_dev *mdiodev = mdio_alloc();
  if (!mdiodev)
    return -ENOMEM;
  strncpy(mdiodev->name, dev->name, MDIO_NAME_LEN);
  mdiodev->read = lowrisc_miiphy_read;
  mdiodev->write = lowrisc_miiphy_write;
  
  retval = mdio_register(mdiodev);
  if (retval < 0)
    return retval;
#endif

  debug("finished initialize\n");
  return 1;
}
