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
char *strcpy(char *dest, const char *src);
char *strncpy(char *dest, const char *src, size_t n);
size_t strlen(const char *s);
void *memcpy(void *dest, const void *src, size_t n);
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
extern int echo;
void myputhex(unsigned n, unsigned width);
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
int sd_transaction(unsigned read, unsigned val, unsigned resp[], unsigned iobuf[], unsigned iobuflen);
void mysleep(int delay);
unsigned int sd_resp(int);
unsigned int sd_stat(int);
void sd_timeout(int d_timeout);
void sd_blksize(int d_blksize);
void sd_blkcnt(int d_blkcnt);
void rx_write_fifo(unsigned int data);
unsigned int rx_read_fifo(void);
void queue_read_array(volatile unsigned int * const sd_ptr, unsigned cnt, unsigned iobuf[]);

void open_handle(void);
void uart_printf(const char *fmt, ...);
void log_printf(const char *fmt, ...);
void uart_write(volatile unsigned int * const sd_ptr, unsigned val);
int cli_readline_into_buffer(const char *const prompt, char *buffer, int timeout);
void sd_transaction_show(void);

extern volatile unsigned int * const sd_base;

  int edcl_main(void);
  int edcl_loadelf(const char *elf);
  void edcl_close(void);
  int edcl_read(uint64_t addr, int bytes, uint8_t *obuf);
  int edcl_write(uint64_t addr, int bytes, uint8_t *ibuf);

/*
 * Controller registers
 */

#define SDHCI_DMA_ADDRESS	0x00

#define SDHCI_BLOCK_SIZE	0x04

#define SDHCI_BLOCK_COUNT	0x06

#define SDHCI_ARGUMENT		0x08

#define SDHCI_TRANSFER_MODE	0x0C
#define  SDHCI_TRNS_DMA		0x01
#define  SDHCI_TRNS_BLK_CNT_EN	0x02
#define  SDHCI_TRNS_ACMD12	0x04
#define  SDHCI_TRNS_READ	0x10
#define  SDHCI_TRNS_MULTI	0x20

#define SDHCI_COMMAND		0x0E
#define  SDHCI_CMD_RESP_MASK	0x03
#define  SDHCI_CMD_CRC		0x08
#define  SDHCI_CMD_INDEX	0x10
#define  SDHCI_CMD_DATA		0x20
#define  SDHCI_CMD_ABORTCMD	0xC0

#define  SDHCI_CMD_RESP_NONE	0x00
#define  SDHCI_CMD_RESP_LONG	0x01
#define  SDHCI_CMD_RESP_SHORT	0x02
#define  SDHCI_CMD_RESP_SHORT_BUSY 0x03

#define SDHCI_MAKE_CMD(c, f) (((c & 0xff) << 8) | (f & 0xff))
#define SDHCI_GET_CMD(c) ((c>>8) & 0x3f)

#define SDHCI_RESPONSE		0x10

#define SDHCI_BUFFER		0x20

#define SDHCI_PRESENT_STATE	0x24
#define  SDHCI_CMD_INHIBIT	0x00000001
#define  SDHCI_DATA_INHIBIT	0x00000002
#define  SDHCI_DOING_WRITE	0x00000100
#define  SDHCI_DOING_READ	0x00000200
#define  SDHCI_SPACE_AVAILABLE	0x00000400
#define  SDHCI_DATA_AVAILABLE	0x00000800
#define  SDHCI_CARD_PRESENT	0x00010000
#define  SDHCI_CARD_STATE_STABLE	0x00020000
#define  SDHCI_CARD_DETECT_PIN_LEVEL	0x00040000
#define  SDHCI_WRITE_PROTECT	0x00080000

#define SDHCI_HOST_CONTROL	0x28
#define  SDHCI_POWER_ON		0x01
#define  SDHCI_CTRL_LED		0x01
#define  SDHCI_CTRL_4BITBUS	0x02
#define  SDHCI_CTRL_HISPD	0x04
#define  SDHCI_CTRL_DMA_MASK	0x18
#define   SDHCI_CTRL_SDMA	0x00
#define   SDHCI_CTRL_ADMA1	0x08
#define   SDHCI_CTRL_ADMA32	0x10
#define   SDHCI_CTRL_ADMA64	0x18
#define  SDHCI_CTRL_8BITBUS	0x20
#define  SDHCI_CTRL_CD_TEST_INS	0x40
#define  SDHCI_CTRL_CD_TEST	0x80

#define SDHCI_POWER_CONTROL	0x29
#define  SDHCI_POWER_180	0x0A
#define  SDHCI_POWER_300	0x0C
#define  SDHCI_POWER_330	0x0E

#define SDHCI_BLOCK_GAP_CONTROL	0x2A
#define SDHCI_WAKE_UP_CONTROL	0x2B
#define SDHCI_TIMEOUT_CONTROL	0x2E
#define SDHCI_SOFTWARE_RESET	0x2F

#define  SDHCI_WAKE_ON_INT	0x01
#define  SDHCI_WAKE_ON_INSERT	0x02
#define  SDHCI_WAKE_ON_REMOVE	0x04

#define SDHCI_CLOCK_CONTROL	0x2C
#define  SDHCI_DIVIDER_SHIFT	8
#define  SDHCI_DIV_MASK	        0xFFFFFF
#define  SDHCI_CLOCK_CARD_EN	0x0004
#define  SDHCI_CLOCK_INT_STABLE	0x0002
#define  SDHCI_CLOCK_INT_EN	0x0001

#define  SDHCI_RESET_ALL	0x01
#define  SDHCI_RESET_CMD	0x02
#define  SDHCI_RESET_DATA	0x04

#define SDHCI_INT_STATUS	0x30
#define SDHCI_INT_ENABLE	0x34
#define SDHCI_SIGNAL_ENABLE	0x38
#define  SDHCI_INT_RESPONSE	0x00000001
#define  SDHCI_INT_DATA_END	0x00000002
#define  SDHCI_INT_DMA_END	0x00000008
#define  SDHCI_INT_SPACE_AVAIL	0x00000010
#define  SDHCI_INT_DATA_AVAIL	0x00000020
#define  SDHCI_INT_CARD_INSERT	0x00000040
#define  SDHCI_INT_CARD_REMOVE	0x00000080
#define  SDHCI_INT_CARD_INT	0x00000100
#define  SDHCI_INT_ERROR	0x00008000
#define  SDHCI_INT_TIMEOUT	0x00010000
#define  SDHCI_INT_CRC		0x00020000
#define  SDHCI_INT_END_BIT	0x00040000
#define  SDHCI_INT_INDEX	0x00080000
#define  SDHCI_INT_DATA_TIMEOUT	0x00100000
#define  SDHCI_INT_DATA_CRC	0x00200000
#define  SDHCI_INT_DATA_END_BIT	0x00400000
#define  SDHCI_INT_BUS_POWER	0x00800000
#define  SDHCI_INT_ACMD12ERR	0x01000000
#define  SDHCI_INT_ADMA_ERROR	0x02000000

#define  SDHCI_INT_NORMAL_MASK	0x00007FFF
#define  SDHCI_INT_ERROR_MASK	0xFFFF8000

#define  SDHCI_INT_CMD_MASK	(SDHCI_INT_RESPONSE | SDHCI_INT_TIMEOUT | \
		SDHCI_INT_CRC | SDHCI_INT_END_BIT | SDHCI_INT_INDEX)
#define  SDHCI_INT_DATA_MASK	(SDHCI_INT_DATA_END | SDHCI_INT_DMA_END | \
		SDHCI_INT_DATA_AVAIL | SDHCI_INT_SPACE_AVAIL | \
		SDHCI_INT_DATA_TIMEOUT | SDHCI_INT_DATA_CRC | \
		SDHCI_INT_DATA_END_BIT | SDHCI_INT_ADMA_ERROR)
#define SDHCI_INT_ALL_MASK	((unsigned int)-1)

#define SDHCI_ACMD12_ERR	0x3C
#define SDHCI_HOST_CONTROL2	0x3E

/* 3E-3F reserved */

#define SDHCI_CAPABILITIES	0x40
#define  SDHCI_TIMEOUT_CLK_MASK	0x0000003F
#define  SDHCI_TIMEOUT_CLK_SHIFT 0
#define  SDHCI_TIMEOUT_CLK_UNIT	0x00000080
#define  SDHCI_CLOCK_BASE_MASK	0x00003F00
#define  SDHCI_CLOCK_V3_BASE_MASK	0x0000FF00
#define  SDHCI_CLOCK_BASE_SHIFT	8
#define  SDHCI_MAX_BLOCK_MASK	0x00030000
#define  SDHCI_MAX_BLOCK_SHIFT  16
#define  SDHCI_CAN_DO_8BIT	0x00040000
#define  SDHCI_CAN_DO_ADMA2	0x00080000
#define  SDHCI_CAN_DO_ADMA1	0x00100000
#define  SDHCI_CAN_DO_HISPD	0x00200000
#define  SDHCI_CAN_DO_SDMA	0x00400000
#define  SDHCI_CAN_VDD_330	0x01000000
#define  SDHCI_CAN_VDD_300	0x02000000
#define  SDHCI_CAN_VDD_180	0x04000000
#define  SDHCI_CAN_64BIT	0x10000000

#define SDHCI_CAPABILITIES_1	0x44
#define  SDHCI_CLOCK_MUL_MASK	0x00FF0000
#define  SDHCI_CLOCK_MUL_SHIFT	16

#define SDHCI_MAX_CURRENT	0x48

/* 4C-4F reserved for more max current */

#define SDHCI_SET_ACMD12_ERROR	0x50
#define SDHCI_SET_INT_ERROR	0x52

#define SDHCI_ADMA_ERROR	0x54

/* 55-57 reserved */

#define SDHCI_ADMA_ADDRESS	0x58

/* 60-FB reserved */

#define SDHCI_SLOT_INT_STATUS	0xFC

#define SDHCI_HOST_VERSION	0xFE
#define  SDHCI_VENDOR_VER_MASK	0xFF00
#define  SDHCI_VENDOR_VER_SHIFT	8
#define  SDHCI_SPEC_VER_MASK	0x00FF
#define  SDHCI_SPEC_VER_SHIFT	0
#define   SDHCI_SPEC_100	0
#define   SDHCI_SPEC_200	1
#define   SDHCI_SPEC_300	2

#define MMC_CAP2_NO_SDIO	(1 << 19)

#define SDHCI_GET_VERSION(x) (x->version & SDHCI_SPEC_VER_MASK)
#define get_card_status(_verbose) _get_card_status(__LINE__, _verbose)

/*
 * End of controller registers.
 */

#define SDHCI_MAX_DIV_SPEC_200	256
#define SDHCI_MAX_DIV_SPEC_300	2046

struct minion_uart_host {
	const char *name;
	unsigned int quirks;
	unsigned int host_caps;
	unsigned int version;
	unsigned int clock;
	struct mmc *mmc;
	const struct minion_uart_ops *ops;
	int index;
	int bus_width;
	uint	voltages;
        uint32_t *start_addr;
};

extern char minion_iobuf[512];

void myputchar(char ch);
void myputs(const char *str);
void minion_uart_write(struct minion_uart_host *host, uint32_t val, int reg);
uint32_t minion_uart_read(struct minion_uart_host *host, int reg);
void minion_uart_reset(struct minion_uart_host *host, uint8_t mask);
//void minion_uart_cmd_done(struct minion_uart_host *host, uint resp_type, uint cmd_response[]);
int sd_flush(unsigned iobuf[], unsigned iobuflen, unsigned trans);
void minion_dispatch(const char *ucmd);

/*
 * Host SDMA buffer boundary. Valid values from 4K to 512K in powers of 2.
 */
#define SDHCI_DEFAULT_BOUNDARY_SIZE	(512 * 1024)
#define SDHCI_DEFAULT_BOUNDARY_ARG	(7)
struct minion_uart_ops {
#ifdef CONFIG_MMC_SDHCI_IO_ACCESSORS
	uint32_t             (*read_l)(struct minion_uart_host *host, int reg);
	uint16_t             (*read_w)(struct minion_uart_host *host, int reg);
	uint8_t              (*read_b)(struct minion_uart_host *host, int reg);
	void            (*write_l)(struct minion_uart_host *host, uint32_t val, int reg);
	void            (*write_w)(struct minion_uart_host *host, uint16_t val, int reg);
	void            (*write_b)(struct minion_uart_host *host, uint8_t val, int reg);
#endif
};

/*
 * No command will be sent by driver if card is busy, so driver must wait
 * for card ready state.
 * Every time when card is busy after timeout then (last) timeout value will be
 * increased twice but only if it doesn't exceed global defined maximum.
 * Each function call will use last timeout value.
 */
#define SDHCI_CMD_MAX_TIMEOUT			3200
#define SDHCI_CMD_DEFAULT_TIMEOUT		100
#define SDHCI_READ_STATUS_TIMEOUT		1000
#endif
