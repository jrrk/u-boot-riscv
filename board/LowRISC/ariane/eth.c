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

#if 1
static inline u64 lowrisc_reg_read(struct eth_device *dev, u32 offset)
{
  if (0)
	return *(volatile u64*)(dev->iobase + offset);
  else
    {
      debug("lowrisc_reg_read(%lX,%X)\n", dev->iobase, offset);
      return 0;
    }
}

static inline void lowrisc_reg_write(struct eth_device *dev, u32 offset, u64 val)
{
  if (0)
	*(volatile u64*)(dev->iobase + offset) = val;
  else
    {
      debug("lowrisc_reg_write(%lX,%X,%llX)\n", dev->iobase, offset, val);
    }
}
#endif

static u64 lowrisc_get_mac_csr(struct eth_device *dev, u8 reg)
{
  return 0;
}

static void lowrisc_set_mac_csr(struct eth_device *dev, u8 reg, u64 data)
{

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

	addrl = m[0] | (m[1] << 8) | (m[2] << 16) | (m[3] << 24);
	addrh = m[4] | (m[5] << 8);
        //	lowrisc_set_mac_csr(dev, ADDRL, addrl);
        //	lowrisc_set_mac_csr(dev, ADDRH, addrh);

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

}

static int lowrisc_init(struct eth_device *dev, bd_t * bd)
{
	struct chip_id *id = dev->priv;

	printf(DRIVERNAME ": detected %s controller\n", id->name);

	return 0;
}

static int lowrisc_send(struct eth_device *dev, void *packet, int length)
{
	u32 *data = (u32*)packet;
	u32 tmplen;
	u32 status;

        return 0;
}

static void lowrisc_halt(struct eth_device *dev)
{
	lowrisc_reset(dev);
	lowrisc_handle_mac_address(dev);
}

static int lowrisc_rx(struct eth_device *dev)
{
	u32 *data = (u32 *)net_rx_packets[0];
	u32 pktlen, tmplen;
	u32 status;

        pktlen = 64;
        net_process_received_packet(net_rx_packets[0], pktlen);

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
  
  debug("dev->iobase = %x\n", dev->iobase);
        
  /* Try to detect chip. Will fail if not present. */
  if (lowrisc_detect_chip(dev)) {
    free(dev);
    return 0;
  }

#if 0
  addrh = lowrisc_get_mac_csr(dev, ADDRH);
  addrl = lowrisc_get_mac_csr(dev, ADDRL);
#endif  
  debug("MAC addr = %X:%X\n", addrh, addrl);
  if (!(addrl == 0xffffffff && addrh == 0x0000ffff)) {
    /* address is obtained from optional eeprom */
    dev->enetaddr[0] = addrl;
    dev->enetaddr[1] = addrl >>  8;
    dev->enetaddr[2] = addrl >> 16;
    dev->enetaddr[3] = addrl >> 24;
    dev->enetaddr[4] = addrh;
    dev->enetaddr[5] = addrh >> 8;
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
