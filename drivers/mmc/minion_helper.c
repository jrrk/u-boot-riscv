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
#include <stdint.h>
#include "minion_lib.h"

static int uart_handle, log_handle, bufptr;
static char readbuf[999];

void myexit(int status)
{
  exit(status);
}

void myperror(const char *s)
{
  perror(s);
}

void myassert(int cond)
{
  ((cond) ? (void) (0) : __assert_fail ("cond", "drivers/mmc/minion_helper.c", 77, __PRETTY_FUNCTION__));
}

int fionread(unsigned *cmd, unsigned *arg, unsigned *len, unsigned *resp)
{
  int rslt, cnt, maxcnt = sizeof(readbuf) - bufptr;
  usleep(10000);
  rslt = ioctl(uart_handle, FIONREAD, &cnt);
  if (rslt < 0)
    {
      perror("uart ioctl error");
      myexit(1);
    }
  if (cnt > maxcnt) cnt = maxcnt;
  if (cnt > 0)
    {
      rslt = read(uart_handle, readbuf+bufptr, cnt);
      if (rslt < 0)
	{
	  perror("uart read error");
	  myexit(1);
	}
      if (cnt > rslt) cnt = rslt;
    }
  if (cnt > 0) write(log_handle, readbuf+bufptr, cnt);
  bufptr += cnt;
  readbuf[bufptr] = 0;
  rslt = sscanf(readbuf, "\ns %X,%X,%X\r\n\r\n%X:%X->%X,%X,%X,%X,%X,%X\n", cmd, arg, len, resp+7, resp+6, resp+3, resp+2, resp+1, resp+0, resp+5, resp+4);
  if (rslt == 11)
    return 11;
  rslt = sscanf(readbuf, "\nW %X,%X\n", cmd, arg);
  if (rslt == 2)
    return 2;
  rslt = sscanf(readbuf, "\nR %X:%X\n", cmd, arg);
  if (rslt == 2)
    return 2;
  rslt = !strcmp(readbuf, "\nE\n");
  if (rslt == 1)
    return 1;
  return 0;
}

void open_handle(void)
{
  int rslt;
  struct termios oldt;
  if (uart_handle) close(uart_handle);
  if (!log_handle) log_handle = open("minion_uart.log", O_CREAT|O_TRUNC|O_WRONLY, 0600);
  if (uart_handle < 0)
    {
      perror("uart device error");
      myexit(1);
    }
  uart_handle = open("/dev/ttyUSB1", O_RDWR);
  if (uart_handle < 0)
    {
      perror("uart device error");
      myexit(1);
    }
  tcflush(uart_handle, TCIOFLUSH);
  rslt = tcgetattr(uart_handle, &oldt);
  if (rslt)
    {
      perror("tcgetattr");
      myexit(2);
    }
  cfsetispeed(&oldt, B115200);
  cfsetospeed(&oldt, B115200);
  cfmakeraw(&oldt);
  rslt = tcsetattr(uart_handle, 0, &oldt);
  if (rslt)
    {
      perror("tcsetattr");
      myexit(3);
    }
  
}

void uart_printf(const char *fmt, ...)
{
  int i, rslt;
  va_list args;
  char printbuffer[256];
  va_start(args, fmt);
  i = vsnprintf(printbuffer, sizeof(printbuffer), fmt, args);
  va_end(args);
  tcflush(uart_handle, TCIOFLUSH);
  rslt = write(uart_handle, printbuffer, i);
  myassert(rslt==i);
  tcdrain(uart_handle);
  bufptr = 0; // pending incoming characters are flushed
}

void log_printf(const char *fmt, ...)
{
  int i, rslt;
  va_list args;
  char printbuffer[256];
  va_start(args, fmt);
  i = vsnprintf(printbuffer, sizeof(printbuffer), fmt, args);
  va_end(args);
  rslt = write(log_handle, printbuffer, i);
  myassert(rslt==i);
  rslt = write(1, printbuffer, i);
}

volatile unsigned int * const led_base = (volatile unsigned int*)(7<<20);
volatile unsigned int * const sd_base = (volatile unsigned int*)(6<<20);
volatile unsigned int * const sd_stat_ = (volatile unsigned int*)(5<<20);

static int led_flag;

void uart_write(volatile unsigned int * const sd_ptr, unsigned val)
 {
   unsigned addr, data, len, tmp_resp[8], cnt = 100;
   uart_printf("W%.6X,%.8X\r", sd_ptr, val);
   while (cnt--)
     {
       if (fionread(&addr, &data, &len, tmp_resp) == 2) cnt = 0;
     }
   if ((addr!=(unsigned)sd_ptr) || (data!=val))
     printf("uart response error\n");
 }

unsigned uart_read(volatile unsigned int * const sd_ptr)
 {
   unsigned addr, data, len, tmp_resp[8];
   uart_printf("R%.6X,%.6X\r", sd_ptr, sd_ptr);
   while (fionread(&addr, &data, &len, tmp_resp) < 2);
   return data;
 }

void o_led(unsigned int flag, unsigned int data)
{
  led_flag = flag;
  uart_write(led_base, (flag << 12)|data);  
}

unsigned int sd_resp(int sel)
{
  unsigned int rslt = uart_read(sd_base+sel);
  return rslt;
}

unsigned int sd_stat(int sel)
{
  unsigned int rslt = uart_read(sd_stat_+sel);
  return rslt;
}

void sd_align(int d_align)
{
    uart_write(sd_base+0, d_align);
}
  
void sd_clk_div(int clk_div)
{
  uart_write(sd_base+1, clk_div);
}

void sd_cmd(unsigned cmd, unsigned arg)
{
  uart_write(sd_base+2, arg);
  uart_write(sd_base+3, cmd);
}

void sd_cmd_setting(int sd_cmd_setting)
{
  uart_write(sd_base+4, sd_cmd_setting);
}

void sd_cmd_start(int sd_cmd)
{
uart_write(sd_base+5, sd_cmd);
}

void sd_reset(int sd_rst, int clk_rst, int data_rst, int cmd_rst)
{
uart_write(sd_base+6, ((sd_rst&1) << 3)|((clk_rst&1) << 2)|((data_rst&1) << 1)|((cmd_rst&1) << 0));
}

void sd_blkcnt(int d_blkcnt)
{
uart_write(sd_base+7, d_blkcnt&0xFFFF);
}

void sd_blksize(int d_blksize)
{
  uart_write(sd_base+8, d_blksize&0xFFF);
}

void sd_timeout(int d_timeout)
{
  uart_write(sd_base+9, d_timeout);
}

void mysleep(int delay)
{
  while (delay--) o_led(led_flag, led_flag);
}

void sd_transaction(int cmd, unsigned arg, unsigned setting, unsigned resp[])
  {
    int i, mask = setting > 7 ? 0x500 : 0x100;
    sd_cmd(cmd,arg);
    sd_cmd_setting(setting);
    o_led(2,0);
    mysleep(10);
    sd_cmd_start(1);
    o_led(3,0);
    while ((sd_stat(0) & mask) != mask);
    o_led(4,0);
    mysleep(10);
    for (i = 8; i--; ) resp[i] = sd_resp(i);
    if (setting > 7)
      {
	int ready = sd_stat(0);
	while (1 & ~ready)
	  {
	    
	    ready = sd_stat(0);
	  }
      }
    o_led(5,0);
    sd_cmd_start(0);
    sd_cmd_setting(0);
    while ((sd_stat(0) & mask) != 0);
    o_led(6,0);
  }

void spi_init(void)
{
  o_led(1,0);
  sd_clk_div(200);
  sd_reset(0,1,0,0);
  mysleep(74);
  sd_blkcnt(1);
  sd_blksize(1);
  sd_align(3);
  sd_timeout(14);
  mysleep(10);
  sd_reset(0,1,1,1);
  mysleep(10);
}

void spi_disable(void) {
  o_led(2,0);
}

uint8_t spi_send(uint8_t dat) {
  o_led(3,dat);
  *(volatile unsigned int*)(5<<20) = dat;
  //  spi_xfer(1, 0, 3, 0, 1);
  *(volatile unsigned int*)(4<<20) = 0;
  return *(volatile unsigned int*)(4<<20);  
}

uint8_t spi_recv(void) {
  o_led(5,0);
  //  spi_xfer(1, 0, 3, 1, 0);
  *(volatile unsigned int*)(4<<20) = 0;
  return *(volatile unsigned int*)(4<<20);  
}

void spi_send_multi(const uint8_t* dat, uint8_t n) {
  uint8_t i;
  o_led(6,0);
  for(i=0; i<n; i++) *(volatile unsigned int*)(5<<20) = *(dat++);
  //  spi_xfer(1, n, 3, 0, 1);
}

void spi_recv_multi(uint8_t* dat, uint8_t n) {
  uint8_t i;
  o_led(7,0);
  //  spi_xfer(1, n, 3, 1, 0);
  for(i=0; i<n; i++)
    {
      *(volatile unsigned int*)(4<<20) = 0;
      *dat++ = *(volatile unsigned int*)(4<<20);  
    }
}

void spi_select_slave(uint8_t id) {
  o_led(8,0);
}

void spi_deselect_slave(uint8_t id) {
  o_led(9,0);
}

