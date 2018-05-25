/*
 * LowRISC Network driver
 *
 * Based on SMSC LAN driver (c) 2007 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _LOWRISC_H_
#define _LOWRISC_H_

#include <linux/types.h>

#define DRIVERNAME "eth"

struct chip_id {
	u16 id;
	char *name;
};

static const struct chip_id chip_ids[] =  {
	{ 0xDEAD, "LOWRISC" },
	{ 0, NULL },
};

/* Register offsets for the LowRISC Core */

#define TXBUFF_OFFSET       0x2000          /* Transmit Buffer */

#define MACLO_OFFSET        0x1000          /* MAC address low 32-bits */
#define MACHI_OFFSET        0x1008          /* MAC address high 16-bits and MAC ctrl */
#define TPLR_OFFSET         0x1010          /* Tx packet length */
#define TFCS_OFFSET         0x1018          /* Tx frame check sequence register */
#define MDIOCTRL_OFFSET     0x1020          /* MDIO Control Register */
#define RFCS_OFFSET         0x1028          /* Rx frame check sequence register */
#define RSR_OFFSET          0x1030          /* Rx status and reset register */
#define RPLR_OFFSET         0x1038          /* Rx packet length register */

#define RXBUFF_OFFSET       0x0000          /* Receive Buffer */
#define MDIORD_RDDATA_MASK    0x0000FFFF    /* Data to be Read */

/* MAC Ctrl Register (MACHI) Bit Masks */
#define MACHI_MACADDR_MASK    0x0000FFFF     /* MAC high 16-bits mask */
#define MACHI_COOKED_MASK     0x00010000     /* obsolete flag */
#define MACHI_LOOPBACK_MASK   0x00020000     /* Rx loopback packets */
#define MACHI_ALLPKTS_MASK    0x00200000     /* Rx all packets (promiscuous mode) */
#define MACHI_IRQ_EN          0x00400000     /* Rx packet interrupt enable */

/* MDIO Control Register Bit Masks */
#define MDIOCTRL_MDIOCLK_MASK 0x00000001    /* MDIO Clock Mask */
#define MDIOCTRL_MDIOOUT_MASK 0x00000002    /* MDIO Output Mask */
#define MDIOCTRL_MDIOOEN_MASK 0x00000004    /* MDIO Output Enable Mask */
#define MDIOCTRL_MDIORST_MASK 0x00000008    /* MDIO Input Mask */
#define MDIOCTRL_MDIOIN_MASK  0x00000008    /* MDIO Input Mask */

/* Transmit Status Register (TPLR) Bit Masks */
#define TPLR_FRAME_ADDR_MASK  0x0FFF0000     /* Tx frame address */
#define TPLR_PACKET_LEN_MASK  0x00000FFF     /* Tx packet length */
#define TPLR_BUSY_MASK        0x80000000     /* Tx busy mask */

/* Receive Status Register (RSR) */
#define RSR_RECV_DONE_MASK    0x00000001      /* Rx complete */
#define RSR_RECV_IRQ_MASK     0x00000002      /* Rx irq bit */

/* Receive Packet Length Register (RPLR) */
#define RPLR_LENGTH_MASK      0x00000FFF      /* Rx packet length */
#define RPLR_ERROR_MASK       0x40000000      /* Rx error mask */
#define RPLR_FCS_ERROR_MASK   0x80000000      /* Rx FCS error mask */

/* General Ethernet Definitions */
#define HEADER_OFFSET               12      /* Offset to length field */
#define HEADER_SHIFT                16      /* Shift value for length */
#define ARP_PACKET_SIZE             28      /* Max ARP packet size */
#define HEADER_IP_LENGTH_OFFSET     16      /* IP Length Offset */

#endif
