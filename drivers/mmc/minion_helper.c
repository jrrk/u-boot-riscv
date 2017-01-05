#ifdef PROTOTYPE
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <assert.h>
#include <sys/ioctl.h>
#else

typedef long int ssize_t;
typedef long unsigned int size_t;

#endif
#include <stdarg.h>
#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <malloc.h>
#include <errno.h>
#include <mmc.h>
#include "minion_lib.h"

volatile unsigned int * const led_base = (volatile unsigned int*)(7<<20);
volatile unsigned int * const sd_base = (volatile unsigned int*)(6<<20);
volatile unsigned int * const sd_stat_ = (volatile unsigned int*)(5<<20);
volatile unsigned int * const rxfifo_base = (volatile unsigned int*)(4<<20);

enum edcl_mode {edcl_mode_unknown, edcl_mode_read, edcl_mode_write, edcl_mode_block_read, edcl_max=256};

#pragma pack(4)

static struct etrans {
  enum edcl_mode mode;
  volatile uint32_t *ptr;
  uint32_t val;
} edcl_trans[edcl_max+1];

#pragma pack()

static int edcl_cnt;

void queue_flush(void)
{
  struct etrans tmp;
  tmp.val = 0xDEADBEEF;
  edcl_trans[edcl_cnt++].mode = edcl_mode_unknown;
  edcl_write(0, edcl_cnt*sizeof(struct etrans), (uint8_t*)edcl_trans);
  edcl_write(edcl_max*sizeof(struct etrans), sizeof(struct etrans), (uint8_t*)&tmp);
  do {
    edcl_read(0, sizeof(tmp), (uint8_t *)&(tmp));
  } while (tmp.ptr);
}

void queue_write(volatile unsigned int *const sd_ptr, unsigned val, int flush)
 {
   struct etrans tmp;
   tmp.mode = edcl_mode_write;
   tmp.ptr = sd_ptr;
   tmp.val = val;
   edcl_trans[edcl_cnt++] = tmp;
   if (flush || (edcl_cnt==edcl_max))
     {
       queue_flush();
       edcl_cnt = 0;
     }
 }

unsigned queue_read(volatile unsigned int * const sd_ptr)
 {
   struct etrans tmp;
   tmp.mode = edcl_mode_read;
   tmp.ptr = sd_ptr;
   tmp.val = 0xDEADBEEF;
   edcl_trans[edcl_cnt++] = tmp;
   queue_flush();
   edcl_read((edcl_cnt-2)*sizeof(struct etrans), sizeof(tmp), (uint8_t *)&tmp);
   edcl_cnt = 0;
   return tmp.val;
 }

void my_led(unsigned int data)
{
  queue_write(led_base, data, 1);  
}

void rx_write_fifo(unsigned int data)
{
  queue_write(rxfifo_base, data, 0);  
}

unsigned int rx_read_fifo(void)
{
  return queue_read(rxfifo_base);
}

unsigned int sd_resp(int sel)
{
  unsigned int rslt = queue_read(sd_base+sel);
  return rslt;
}

unsigned int sd_stat(int sel)
{
  unsigned int rslt = queue_read(sd_stat_+sel);
  return rslt;
}

void sd_align(int d_align)
{
  queue_write(sd_base+0, d_align, 0);
}
  
void sd_clk_div(int clk_div)
{
  queue_write(sd_base+1, clk_div, 0);
}

void sd_cmd(unsigned cmd)
{
  queue_write(sd_base+3, cmd, 0);
}

void sd_arg(unsigned arg)
{
  queue_write(sd_base+2, arg, 0);
}

void sd_setting(int setting)
{
  queue_write(sd_base+4, setting, 0);
}

void sd_cmd_start(int sd_cmd)
{
  queue_write(sd_base+5, sd_cmd, 0);
}

void sd_reset(int sd_rst, int clk_rst, int data_rst, int cmd_rst)
{
  queue_write(sd_base+6, ((sd_rst&1) << 3)|((clk_rst&1) << 2)|((data_rst&1) << 1)|((cmd_rst&1) << 0), 0);
}

void sd_blkcnt(int d_blkcnt)
{
  queue_write(sd_base+7, d_blkcnt&0xFFFF, 0);
}

void sd_blksize(int d_blksize)
{
  queue_write(sd_base+8, d_blksize&0xFFF, 0);
}

void sd_timeout(int d_timeout)
{
  queue_write(sd_base+9, d_timeout, 0);
}

void mysleep(int delay)
{
}

int sd_flush(unsigned iobuf[], unsigned iobuflen, unsigned trans)
{
  int i, cnt = 0;
#if 0  
  int ready = sd_stat(0);
  int itm, discard = 0;
  while (1 & ~ready)
    {
      queue_write(rxfifo_base, 0, 0);
      itm = queue_read(rxfifo_base);
#ifdef CONFIG_MINION_VERBOSE
      if (itm && (itm != -1)) printf("rx_fifo read @%X: %.8X\n", cnt, itm);
#endif
      if (cnt < iobuflen) iobuf[cnt++] = itm; else discard++;
      ready = sd_stat(0);
    }
#else
   struct etrans tmp;
   queue_flush();
   tmp.mode = edcl_mode_block_read;
   tmp.ptr = rxfifo_base;
   tmp.val = 1;
   edcl_write(0, sizeof(struct etrans), (uint8_t*)&tmp);
   tmp.val = 0xDEADBEEF;
   edcl_write(edcl_max*sizeof(struct etrans), sizeof(struct etrans), (uint8_t*)&tmp);
   do {
    edcl_read(0, sizeof(tmp), (uint8_t *)&tmp);
  } while (tmp.ptr);
   cnt = tmp.mode;
   if (cnt > iobuflen) cnt = iobuflen;
   if (cnt) edcl_read(sizeof(struct etrans), cnt*sizeof(uint32_t), (uint8_t *)iobuf);
#endif
#ifdef LEGACY
  if (cnt) iobuf[cnt] = 0;
  for (i = 0; i < cnt; i++)
    iobuf[i] = (iobuf[i] << 24) | (iobuf[i+1] >> 8);
#endif
#ifdef CONFIG_MINION_VERBOSE
  printf("rx_fifo read: %d items (%d transactions, %d discarded)\n", cnt, trans, discard);
#endif
   return cnt;  
}

void board_mmc_power_init(void)
{
  my_led(1);
  sd_clk_div(200);
  sd_reset(0,1,0,0);
  mysleep(74);
  sd_blkcnt(1);
  sd_blksize(1);
#ifdef LEGACY
  sd_align(3);
#else
  sd_align(0);
#endif  
  sd_timeout(14);
  mysleep(10);
  sd_reset(0,1,1,1);
  mysleep(10);
}

const char *scan(const char *start, size_t *data, int base)
{
  *data = 0;
  while (*start)
    {
      if (*start >= '0' && *start <= '9') *data = *data * base + *start++ - '0';
      else if (*start >= 'A' && *start <= 'F') *data = *data * base + *start++ - 'A' + 10;
      else if (*start >= 'a' && *start <= 'f') *data = *data * base + *start++ - 'a' + 10;
      else if (*start == ' ') ++start;
      else return start+1;
    }
  return start;
}

size_t mystrtol(const char *nptr, char **endptr, int base)
{
  size_t data;  
  const char *last = scan(nptr, &data, base);
  if (endptr) *endptr = (char *)last;
  return data;
}

static int minion_uart_host_control;
static int minion_uart_ctrl_dma;
static int minion_uart_ctrl_cd;
static int minion_uart_power_control;
static int minion_uart_power_180;
static int minion_uart_block_gap;
static int minion_uart_wake_up;
static int minion_uart_timeout_control;
static int minion_uart_software_reset;
static int minion_uart_clock_div;
static int minion_uart_int_status;
static int minion_uart_int_enable;
static int minion_uart_signal_enable;
static int minion_uart_present_state;
static int minion_uart_max_current;
static int minion_uart_set_acmd12;
static int minion_uart_set_int;
static int minion_uart_slot_int;
static int minion_uart_host_version;
static int minion_uart_transfer_mode;

void sd_cmd_setting(int cmd_flags)
{
  int setting = 0;
  switch(cmd_flags & MINION_UART_CMD_RESP_MASK)
      {
      case MINION_UART_CMD_RESP_NONE: setting = 0; break;
      case MINION_UART_CMD_RESP_SHORT: setting = 1; break;
      case MINION_UART_CMD_RESP_SHORT_BUSY: setting = 1; break;
      case MINION_UART_CMD_RESP_LONG: setting = 3; break;
      }
    if (minion_uart_transfer_mode & MINION_UART_TRNS_READ)
      {
	setting |= 0x14;
	if (minion_uart_host_control & MINION_UART_CTRL_4BITBUS) setting |= 0x20;
      }
    sd_setting(setting);
}

void sd_transaction_start(int cmd_flags)
  {
    sd_cmd(cmd_flags >> 8);
    sd_cmd_setting(cmd_flags & 255);
    my_led(2);
    mysleep(10);
    sd_cmd_start(1);
    my_led(3);
  }

void sd_transaction_wait(int mask)
{
    while ((sd_stat(0) & mask) != mask);
    my_led(4);
    mysleep(10);
}

int sd_transaction_flush(int flush, unsigned iobuf[], unsigned iobuflen)
{
  int i, cnt = 0;
  //  for (i = 10; i--; ) resp[i] = sd_resp(i);
  if (flush) cnt = sd_flush(iobuf, iobuflen, sd_resp(9));
    return cnt;
}

void sd_transaction_finish(int mask)
{
    my_led(5);
    sd_cmd_start(0);
    sd_cmd_setting(0);
    while ((sd_stat(0) & mask) != 0);
    my_led(6);
}

int sd_transaction(unsigned read, unsigned val, unsigned resp[], unsigned iobuf[], unsigned iobuflen)
  {
    int cnt = 0;
    int cmd = val >> 8;
    int i, mask = read ? 0x500 : 0x100;
#if 0
    sd_cmd(val >> 8);
    sd_cmd_setting(val & 255);
    my_led(2);
    mysleep(10);
    sd_cmd_start(1);
    my_led(3);
#else
    sd_transaction_start(val);
#endif
#if 0
    while ((sd_stat(0) & mask) != mask);
    my_led(4);
    mysleep(10);
#else
    sd_transaction_wait(mask);
#endif
#if 0
    for (i = 10; i--; ) resp[i] = sd_resp(i);
    if (read || !cmd)
      cnt = sd_flush(iobuf, cmd ? iobuflen : 0, sd_resp(9));
#else
    cnt = sd_transaction_flush(read || !cmd, iobuf, cmd ? iobuflen : 0);
#endif    
#if 0
    my_led(5);
    sd_cmd_start(0);
    sd_setting(0);
    while ((sd_stat(0) & mask) != 0);
    my_led(6);
#else
    sd_transaction_finish(mask);
#endif    
    return cnt;
  }

unsigned sd_transaction_v(int sdcmd, unsigned arg, unsigned setting)
{
  int i, mask = setting > 7 ? 0x500 : 0x100;
  unsigned resp[10];
  myputchar('\r');
  myputchar('\n');
  sd_arg(arg);
  sd_setting(setting);
  sd_cmd(sdcmd);
  my_led(2);
  mysleep(10);
  sd_cmd_start(1);
  my_led(3);
  sd_transaction_wait(mask);
  for (i = 10; i--; ) resp[i] = sd_resp(i);
  sd_transaction_finish(mask);
  myputhex(resp[7], 4);
  myputchar(':');
  myputhex(resp[6], 8);
  myputchar('-');
  myputchar('>');
  for (i = 4; i--; )
    {
      myputhex(resp[i], 8);
      myputchar(',');
    }
  myputhex(resp[5], 8);
  myputchar(',');
  myputhex(resp[4], 8);
  return resp[0] & 0xFFFF0000U;
}

void minion_dispatch(const char *ucmd)
{
  int i, rca, busy;
  size_t addr, addr2, data, sdcmd, arg, setting;
  const char *nxt;
  switch(*ucmd)
      {
      case 4:
	break;
      case 'e':
	echo = !echo;
	myputchar('\n');
	myputchar('e');
	myputchar(' ');
	myputhex(echo, 1);
	break;
      case 'i':
	nxt = scan(ucmd+1, &addr, 16);
	myputchar('\n');
	myputchar('i');
	myputchar(' ');
	sd_transaction_v(0,0x00000000,0x0);
	sd_transaction_v(8,0x000001AA,0x1);
	do {
	sd_transaction_v(55,0x00000000,0x1);
	busy = sd_transaction_v(41,0x40300000,0x1);
	} while (0x80000000U & ~busy);
	sd_transaction_v(2,0x00000000,0x3);
	rca = sd_transaction_v(3,0x00000000,0x1);
	myputchar('\r');
	myputchar('\n');
	myputchar('c');
	myputhex(rca, 8);
	sd_transaction_v(9,rca,0x3);
	sd_transaction_v(13,rca,0x1);
	sd_transaction_v(7,rca,0x1);
	sd_transaction_v(55,rca,0x1);
	sd_transaction_v(51,0x00000000,0x1);
	sd_transaction_v(55,rca,0x1);
	sd_transaction_v(13,0x00000000,0x1);
	for (i = 0; i < 16; i=(i+1)|1)
	  {
	    sd_transaction_v(16,0x00000200,0x1);
	    sd_transaction_v(17,i,0x1);
	    sd_transaction_v(16,0x00000200,0x1);
	  }
	sd_transaction_v(16,0x00000200,0x1);
	sd_transaction_v(18,0x00000040,0x1);
	sd_transaction_v(12,0x00000000,0x1);
	break;
      case 'r':
	nxt = scan(ucmd+1, &addr, 16);
	myputchar('\n');
	myputchar('r');
	myputchar(' ');
	myputhex(addr, 8);
	myputchar(',');
	data = queue_read((unsigned *)addr);
	myputhex(data, 2);
	break;
      case 'R':
	nxt = scan(ucmd+1, &addr, 16);
	addr &= ~3;
	nxt = scan(nxt, &addr2, 16);
	while (addr <= addr2)
	  {
	    myputchar('\n');
	    myputchar('R');
	    myputchar(' ');
	    myputhex(addr, 8);
	    myputchar(':');
	    data = queue_read((unsigned *)addr);
	    myputhex(data, 8);
	    addr += 4;
	  }
	break;
      case 'W':
	nxt = scan(ucmd+1, &addr, 16);
	myputchar('\n');
	myputchar('W');
	myputchar(' ');
	myputhex(addr, 8);
	nxt = scan(nxt, &data, 16);
	myputchar(',');
	myputhex(data, 8);
	queue_write((unsigned *)addr, data, 0);
	break;
      case 's':
	nxt = scan(ucmd+1, &sdcmd, 16);
	myputchar('\n');
	myputchar('s');
	myputchar(' ');
	myputhex(sdcmd,2);
	nxt = scan(nxt, &arg, 16);
	myputchar(',');
	myputhex(arg, 8);
	nxt = scan(nxt, &setting, 16);
	myputchar(',');
	myputhex(setting, 1);
	myputchar('\r');
        myputchar('\n');
	sd_transaction_v(sdcmd, arg, setting);
	break;
      case 'q':
	break;
      default: myputs("\nunknown command");
      }
}

void minion_uart_write(struct minion_uart_host *host, uint32_t val, int reg)
{  
  int i, len, read, cmd, mask;
  switch (reg)
    {
    case MINION_UART_BLOCK_COUNT	:
      sd_blkcnt(val);
      break;
    case MINION_UART_BLOCK_SIZE	        :
      sd_blksize(val);
      break;
    case MINION_UART_HOST_CONTROL	:
      minion_uart_host_control = val;
      printf("host_control = %d\n", val);
      if (val & MINION_UART_CTRL_4BITBUS)
	printf("4-bit bus enabled\n");
      else
	printf("4-bit bus disabled\n");
      break;
    case MINION_UART_CTRL_DMA_MASK	: minion_uart_ctrl_dma = val; break;
    case MINION_UART_ARGUMENT	        :
      sd_arg(val);
      break;
    case MINION_UART_TRANSFER_MODE	: minion_uart_transfer_mode = val; break;

    case MINION_UART_CTRL_CD_TEST_INS   : minion_uart_ctrl_cd = val; break;
    case MINION_UART_CTRL_CD_TEST	: minion_uart_ctrl_cd = val; break;
    case MINION_UART_POWER_CONTROL	:
      minion_uart_power_control = val;
      printf("power control = %d\n", val);
      break;
    case MINION_UART_POWER_180	        : minion_uart_power_180 = val; break;
    case MINION_UART_COMMAND	        :
      cmd = val >> 8;
      read = minion_uart_transfer_mode & MINION_UART_TRNS_READ;
      mask = read ? 0x500 : 0x100;
      sd_transaction_start(val);
      sd_transaction_wait(mask);
      sd_transaction_finish(mask);
      minion_uart_int_status = MINION_UART_INT_RESPONSE;
      break;
    case MINION_UART_BLOCK_GAP_CONTROL	: minion_uart_block_gap = val; break;
    case MINION_UART_WAKE_UP_CONTROL	: minion_uart_wake_up = val; break;
    case MINION_UART_TIMEOUT_CONTROL	:
      minion_uart_timeout_control = val;
      sd_timeout(minion_uart_timeout_control);
      break;
    case MINION_UART_SOFTWARE_RESET	:
      minion_uart_software_reset = val;
      minion_uart_timeout_control = 1000; 
      minion_uart_transfer_mode = 0;
      break;
    case MINION_UART_CLOCK_CONTROL	:
      minion_uart_clock_div = val >> MINION_UART_DIVIDER_SHIFT;
      printf("Clock divider = %d\n", minion_uart_clock_div);
      if (val & MINION_UART_CLOCK_CARD_EN) printf("Card clock enabled\n"); else printf("Card clock disabled\n");
      break;
    case MINION_UART_INT_STATUS	:
      minion_uart_int_status = val;
      minion_uart_transfer_mode = 0;
      break;
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

uint32_t minion_uart_read(struct minion_uart_host *host, int reg)
{
  switch (reg)
    {
    case MINION_UART_RESPONSE          : return sd_resp(0);
    case MINION_UART_RESPONSE+4        : return sd_resp(1);
    case MINION_UART_RESPONSE+8        : return sd_resp(2);
    case MINION_UART_RESPONSE+12       : return sd_resp(3);
    case MINION_UART_INT_STATUS	:
      return sd_resp(4) < minion_uart_timeout_control ? MINION_UART_INT_RESPONSE|MINION_UART_INT_DATA_AVAIL : MINION_UART_INT_ERROR;
    case MINION_UART_INT_ENABLE	: return minion_uart_int_enable;
    case MINION_UART_PRESENT_STATE	: return MINION_UART_DATA_AVAILABLE;
    case MINION_UART_HOST_VERSION	: return minion_uart_host_version;
    case MINION_UART_CAPABILITIES      : return MINION_UART_CAN_VDD_330;
    case MINION_UART_SOFTWARE_RESET : return 0;
    case MINION_UART_HOST_CONTROL: return minion_uart_host_control;
    case MINION_UART_CLOCK_CONTROL: return (minion_uart_clock_div << MINION_UART_DIVIDER_SHIFT)|MINION_UART_CLOCK_INT_STABLE;
    case MINION_UART_BUFFER : return 0;
    default: printf("unknown(%d)", reg);
    }
  return 0;
}
