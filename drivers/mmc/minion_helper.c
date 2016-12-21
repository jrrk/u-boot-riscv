#ifdef PROTOTYPE
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#else
typedef unsigned char cc_t;
typedef unsigned int speed_t;
typedef unsigned int tcflag_t;
typedef long unsigned int size_t;
typedef long int ssize_t;

struct termios
  {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[32];
    speed_t c_ispeed;
    speed_t c_ospeed;
  };
#endif
#include <stdarg.h>

/* this section is a hack to prevent conflicts between u-boot and system symbols */

void exit(int status);
int open(const char *pathname, int flags, ...);
ssize_t write(int fd, const void *buf, size_t count);
ssize_t read(int fd, void *buf, size_t count);
int ioctl(int fd, unsigned long request, ...);
int close(int fd);
void perror(const char *s);
int sscanf(const char *str, const char *format, ...);
int tcdrain(int fd);
int tcgetattr(int fd, struct termios *termios_p);
void cfmakeraw(struct termios *termios_p);
int cfsetispeed(struct termios *termios_p, speed_t speed);
int cfsetospeed(struct termios *termios_p, speed_t speed);
int tcsetattr(int fd, int optional_actions, const struct termios *termios_p);
int vsnprintf (char * __s, size_t maxlen, const char *format, __gnuc_va_list __arg);


#define O_RDONLY            00
#define O_WRONLY            01
#define O_RDWR              02
#define O_CREAT           0100 /* Not fcntl.  */
#define O_TRUNC          01000 /* Not fcntl.  */

#define B115200  0010002
#define FIONREAD 0x541B

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

int fionread(unsigned *cmd, unsigned *arg, unsigned *len, unsigned *resp)
{
  int cnt, maxcnt = sizeof(readbuf) - bufptr;
  int rslt = ioctl(uart_handle, FIONREAD, &cnt);
  if (rslt < 0)
    {
      perror("uart ioctl error");
      myexit(1);
    }
  if (cnt > maxcnt) cnt = maxcnt;
  rslt = read(uart_handle, readbuf+bufptr, cnt);
  if (rslt < 0)
    {
      perror("uart read error");
      myexit(1);
    }
  if (cnt > rslt) cnt = rslt;
  write(log_handle, readbuf+bufptr, cnt);
  bufptr += cnt;
  readbuf[bufptr] = 0;
  rslt = sscanf(readbuf, "\ns %X,%X,%X=%X,%X,%X,%X,%X,%X\r\n%X:%X\n", cmd, arg, len, resp+3, resp+2, resp+1, resp+0, resp+5, resp+4, resp+7, resp+6);
  if (rslt == 11)
    return 11;
  return 0;
}

void open_handle(void)
{
  int rslt;
  struct termios oldt;
  if (uart_handle) close(uart_handle);
  log_handle = open("minion_uart.log", O_CREAT|O_TRUNC|O_WRONLY, 0600);
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
  int i;
  va_list args;
  char printbuffer[256];
  va_start(args, fmt);
  i = vsnprintf(printbuffer, sizeof(printbuffer), fmt, args);
  va_end(args);
  write(uart_handle, printbuffer, i);
  tcdrain(uart_handle);
  write(log_handle, printbuffer, i);
  bufptr = 0;
}

