// See LICENSE for license details.

#ifndef MINION_HEADER_H
#define MINION_HEADER_H

/* this section is a hack to prevent conflicts between u-boot and system symbols */

typedef unsigned char cc_t;
typedef unsigned int speed_t;
typedef unsigned int tcflag_t;
typedef unsigned int useconds_t;

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

void exit(int status);
int strcmp(const char *s1, const char *s2);
int open(const char *pathname, int flags, ...);
ssize_t write(int fd, const void *buf, size_t count);
ssize_t read(int fd, void *buf, size_t count);
int ioctl(int fd, unsigned long request, ...);
int close(int fd);
void perror(const char *s);
int sscanf(const char *str, const char *format, ...);
int tcdrain(int fd);
int tcflush(int fd, int queue_selector);
int tcgetattr(int fd, struct termios *termios_p);
void cfmakeraw(struct termios *termios_p);
int cfsetispeed(struct termios *termios_p, speed_t speed);
int cfsetospeed(struct termios *termios_p, speed_t speed);
int tcsetattr(int fd, int optional_actions, const struct termios *termios_p);
int vsnprintf (char * __s, size_t maxlen, const char *format, __gnuc_va_list __arg);
int usleep(useconds_t usec);
void __assert_fail (const char *__assertion, const char *__file, unsigned int __line, const char *__function);

#define O_RDONLY            00
#define O_WRONLY            01
#define O_RDWR              02
#define O_CREAT           0100 /* Not fcntl.  */
#define O_TRUNC          01000 /* Not fcntl.  */

#define B115200   0010002
#define FIONREAD   0x541B
#define TCIOFLUSH       2

// MINION_LIB APIs
extern void puts (const char *);
extern int printf (const char *, ...);
extern void uart_init (void);
extern void uart_send (uint8_t);
extern void uart_send_string (const char *);
extern void uart_send_buf (const char *, const int32_t);
extern uint8_t uart_recv (void);
extern uint8_t uart_read_irq (void);
extern uint8_t uart_check_read_irq (void);
extern void uart_enable_read_irq (void);
extern void uart_disable_read_irq (void);
extern void uart_send (uint8_t data);
extern void uart_init (void);
extern void uart_send_buf (const char *buf, const const int32_t len);
extern uint8_t uart_recv (void);
extern void uart_send_string (const char *str);
extern void cpu_perf_set (unsigned int counterId, unsigned int value);
extern void illegal_insn_handler_c (void);
extern void int_time_cmp (void);
extern void int_main (void);
extern void uart_set_cfg (int parity, uint16_t clk_counter);
extern void __libc_init_array (void);
extern char uart_getchar (void);
extern void uart_wait_tx_done (void);
extern void uart_sendchar (const const char c);
extern void mystatus (void);
// SDCARD entry point
void spi_init(void);
unsigned sd_transaction_v(int sdcmd, unsigned arg, unsigned setting);
void sd_transaction(int cmd, unsigned arg, unsigned setting, unsigned resp[]);
void mysleep(int delay);
unsigned int sd_resp(int);
unsigned int sd_stat(int);

void open_handle(void);
int fionread(unsigned *cmd, unsigned *arg, unsigned *len, unsigned *resp);
void uart_printf(const char *fmt, ...);
void log_printf(const char *fmt, ...);
void uart_write(volatile unsigned int * const sd_ptr, unsigned val);

extern volatile unsigned int * const sd_base;

#endif
