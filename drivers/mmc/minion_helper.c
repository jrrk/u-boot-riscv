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
#include "os.h"
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
#ifdef CONFIG_MINION_VERBOSE
  rslt = write(1, printbuffer, i);
#endif
}

volatile unsigned int * const led_base = (volatile unsigned int*)(7<<20);
volatile unsigned int * const sd_base = (volatile unsigned int*)(6<<20);
volatile unsigned int * const sd_stat_ = (volatile unsigned int*)(5<<20);
volatile unsigned int * const rxfifo_base = (volatile unsigned int*)(4<<20);

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

void uart_write_byte(volatile unsigned char * const sd_ptr, unsigned char val)
 {
   unsigned addr, data, len, tmp_resp[8], cnt = 100;
   uart_printf("w%.6X,%.8X\r", sd_ptr, val);
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
//  while (delay--) o_led(led_flag, led_flag);
}

int sd_flush(unsigned iobuf[], unsigned iobuflen, unsigned trans)
{
  int i, cnt = 0;
  int ready = sd_stat(0);
  int itm, discard = 0;
  while (1 & ~ready)
    {
      uart_write(rxfifo_base, 0);
      itm = uart_read(rxfifo_base);
#ifdef CONFIG_MINION_VERBOSE
      if (itm && (itm != -1)) printf("rx_fifo read @%X: %.8X\n", cnt, itm);
#endif
      if (cnt < iobuflen) iobuf[cnt++] = itm; else discard++;
      ready = sd_stat(0);
    }
  iobuf[cnt] = 0;
  for (i = 0; i < cnt; i++)
    iobuf[i] = (iobuf[i] << 24) | (iobuf[i+1] >> 8);
#ifdef CONFIG_MINION_VERBOSE
  printf("rx_fifo read: %d items (%d transactions, %d discarded)\n", cnt, trans, discard);
#endif
  return cnt;
}

int sd_transaction(int cmd, unsigned arg, unsigned setting, unsigned resp[], unsigned iobuf[], unsigned iobuflen)
  {
    int cnt = 0;
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
    for (i = 10; i--; ) resp[i] = sd_resp(i);
    if ((setting > 7) || !cmd)
      cnt = sd_flush(iobuf, cmd ? iobuflen : 0, resp[9]);
    o_led(5,0);
    sd_cmd_start(0);
    sd_cmd_setting(0);
    while ((sd_stat(0) & mask) != 0);
    o_led(6,0);
    return cnt;
  }

void board_mmc_power_init(void)
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

int echo = 0;

void myputchar(char ch)
{
   os_putc(ch);
}

void myputs(const char *str)
{
  while (*str)
    {
      myputchar(*str++);
    }
}

void myputn(unsigned n)
{
  if (n > 9) myputn(n / 10);
  myputchar(n%10 + '0');
}

void myputhex(unsigned n, unsigned width)
{
  if (width > 1) myputhex(n >> 4, width-1);
  n &= 15;
  if (n > 9) myputchar(n + 'A' - 10);
  else myputchar(n + '0');
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

/* General definition of the S-Record specification */
enum _SRecordDefinitions {
	/* 768 should be plenty of space to read in an S-Record */
	SRECORD_RECORD_BUFF_SIZE = 768,
	/* Offsets and lengths of various fields in an S-Record record */
	SRECORD_TYPE_OFFSET = 1,
	SRECORD_TYPE_LEN = 1,
	SRECORD_COUNT_OFFSET = 2,
	SRECORD_COUNT_LEN = 2,
	SRECORD_ADDRESS_OFFSET = 4,
	SRECORD_CHECKSUM_LEN = 2,
	/* Maximum ascii hex length of the S-Record data field */
	SRECORD_MAX_DATA_LEN = 64,
	/* Maximum ascii hex length of the S-Record address field */
	SRECORD_MAX_ADDRESS_LEN = 8,
	/* Ascii hex length of a single byte */
	SRECORD_ASCII_HEX_BYTE_LEN = 2,
	/* Start code offset and value */
	SRECORD_START_CODE_OFFSET = 0,
	SRECORD_START_CODE = 'S',
};

/**
 * All possible error codes the Motorola S-Record utility functions may return.
 */
enum SRecordErrors {
	SRECORD_OK = 0, 			/**< Error code for success or no error. */
	SRECORD_ERROR_FILE = -1, 		/**< Error code for error while reading from or writing to a file. You may check errno for the exact error if this error code is encountered. */
	SRECORD_ERROR_EOF = -2, 		/**< Error code for encountering end-of-file when reading from a file. */
	SRECORD_ERROR_INVALID_RECORD = -3, 	/**< Error code for error if an invalid record was read. */
	SRECORD_ERROR_INVALID_ARGUMENTS = -4, 	/**< Error code for error from invalid arguments passed to function. */
	SRECORD_ERROR_NEWLINE = -5, 		/**< Error code for encountering a newline with no record when reading from a file. */
};

/**
 * Motorola S-Record Types S0-S9
 */
enum SRecordTypes {
	SRECORD_TYPE_S0 = 0, /**< Header record, although there is an official format it is often made proprietary by third-parties. 16-bit address normally set to 0x0000 and header information is stored in the data field. This record is unnecessary and commonly not used. */
	SRECORD_TYPE_S1, /**< Data record with 16-bit address */
	SRECORD_TYPE_S2, /**< Data record with 24-bit address */
	SRECORD_TYPE_S3, /**< Data record with 32-bit address */
	SRECORD_TYPE_S4, /**< Extension by LSI Logic, Inc. See their specification for more details. */
	SRECORD_TYPE_S5, /**< 16-bit address field that contains the number of S1, S2, and S3 (all data) records transmitted. No data field. */
	SRECORD_TYPE_S6, /**< 24-bit address field that contains the number of S1, S2, and S3 (all data) records transmitted. No data field. */
	SRECORD_TYPE_S7, /**< Termination record for S3 data records. 32-bit address field contains address of the entry point after the S-Record file has been processed. No data field. */
	SRECORD_TYPE_S8, /**< Termination record for S2 data records. 24-bit address field contains address of the entry point after the S-Record file has been processed. No data field. */
	SRECORD_TYPE_S9, /**< Termination record for S1 data records. 16-bit address field contains address of the entry point after the S-Record file has been processed. No data field. */
};

/**
 * Structure to hold the fields of a Motorola S-Record record.
 */
typedef struct {
	size_t address; 			/**< The address field. This can be 16, 24, or 32 bits depending on the record type. */
	uint8_t data[SRECORD_MAX_DATA_LEN/2]; 	/**< The 8-bit array data field, which has a maximum size of 32 bytes. */
	int dataLen; 				/**< The number of bytes of data stored in this record. */
	int type; 				/**< The Motorola S-Record type of this record (S0-S9). */
	uint8_t checksum; 			/**< The checksum of this record. */
} SRecord;

/**
 * Sets all of the record fields of a Motorola S-Record structure.
 * \param type The Motorola S-Record type (integer value of 0 through 9).
 * \param address The 32-bit address of the data. The actual size of the address (16-,24-, or 32-bits) when written to a file depends on the S-Record type.
 * \param data A pointer to the 8-bit array of data.
 * \param dataLen The size of the 8-bit data array.
 * \param srec A pointer to the target Motorola S-Record structure where these fields will be set.
 * \return SRECORD_OK on success, otherwise one of the SRECORD_ERROR_ error codes.
 * \retval SRECORD_OK on success.
 * \retval SRECORD_ERROR_INVALID_ARGUMENTS if the record pointer is NULL, or if the length of the 8-bit data array is out of range (less than zero or greater than the maximum data length allowed by record specifications, see SRecord.data).
*/
int New_SRecord(int type, uint32_t address, const uint8_t *data, int dataLen, SRecord *srec);

/**
 * Prints the contents of a Motorola S-Record structure to stdout.
 * The record dump consists of the type, address, entire data array, and checksum fields of the record.
 * \param srec A pointer to the Motorola S-Record structure.
 * \return Always returns SRECORD_OK (success).
 * \retval SRECORD_OK on success.
*/
void Print_SRecord(const SRecord *srec);

/**
 * Calculates the checksum of a Motorola S-Record SRecord structure.
 * See the Motorola S-Record specifications for more details on the checksum calculation.
 * \param srec A pointer to the Motorola S-Record structure.
 * \return The 8-bit checksum.
*/
uint8_t Checksum_SRecord(const SRecord *srec);

/* Lengths of the ASCII hex encoded address fields of different SRecord types */
static int SRecord_Address_Lengths[] = {
	4, // S0
	4, // S1
	6, // S2
	8, // S3
	8, // S4
	4, // S5
	6, // S6
	8, // S7
	6, // S8
	4, // S9
};

/* Initializes a new SRecord structure that the paramater srec points to with the passed
 * S-Record type, up to 32-bit integer address, 8-bit data array, and size of 8-bit data array. */
int New_SRecord(int type, uint32_t address, const uint8_t *data, int dataLen, SRecord *srec) {
	/* Data length size check, assertion of srec pointer */
	if (dataLen < 0 || dataLen > SRECORD_MAX_DATA_LEN/2 || srec == (void*)0)
		return SRECORD_ERROR_INVALID_ARGUMENTS;

	srec->type = type;
	srec->address = address;
	memcpy(srec->data, data, dataLen);
	srec->dataLen = dataLen;
	srec->checksum = Checksum_SRecord(srec);

	return SRECORD_OK;
}


/* Utility function to read an S-Record from a file */
int Read_SRecord(char recordBuff[], SRecord *srec) {
	/* A temporary buffer to hold ASCII hex encoded data, set to the maximum length we would ever need */
	char hexBuff[SRECORD_MAX_ADDRESS_LEN+1];
	int asciiAddressLen, asciiDataLen, dataOffset, fieldDataCount, i;

	/* Size check for type and count fields */
	if (strlen(recordBuff) < SRECORD_TYPE_LEN + SRECORD_COUNT_LEN)
		return SRECORD_ERROR_INVALID_RECORD;

	/* Check for the S-Record start code at the beginning of every record */
	if (recordBuff[SRECORD_START_CODE_OFFSET] != SRECORD_START_CODE)
		return SRECORD_ERROR_INVALID_RECORD;

	/* Copy the ASCII hex encoding of the type field into hexBuff, convert it into a usable integer */
	strncpy(hexBuff, recordBuff+SRECORD_TYPE_OFFSET, SRECORD_TYPE_LEN);
	hexBuff[SRECORD_TYPE_LEN] = 0;
	srec->type = mystrtol(hexBuff, (char **)0, 16);

	/* Copy the ASCII hex encoding of the count field into hexBuff, convert it to a usable integer */
	strncpy(hexBuff, recordBuff+SRECORD_COUNT_OFFSET, SRECORD_COUNT_LEN);
	hexBuff[SRECORD_COUNT_LEN] = 0;
	fieldDataCount = mystrtol(hexBuff, (char **)0, 16);

	/* Check that our S-Record type is valid */
	if (srec->type < SRECORD_TYPE_S0 || srec->type > SRECORD_TYPE_S9)
		return SRECORD_ERROR_INVALID_RECORD;
	/* Get the ASCII hex address length of this particular S-Record type */
	asciiAddressLen = SRecord_Address_Lengths[srec->type];

	/* Size check for address field */
	if (strlen(recordBuff) < (unsigned int)(SRECORD_ADDRESS_OFFSET+asciiAddressLen))
		return SRECORD_ERROR_INVALID_RECORD;

	/* Copy the ASCII hex encoding of the count field into hexBuff, convert it to a usable integer */
	strncpy(hexBuff, recordBuff+SRECORD_ADDRESS_OFFSET, asciiAddressLen);
	hexBuff[asciiAddressLen] = 0;
	srec->address = mystrtol(hexBuff, (char **)0, 16);

	/* Compute the ASCII hex data length by subtracting the remaining field lengths from the S-Record
	 * count field (times 2 to account for the number of characters used in ASCII hex encoding) */
	asciiDataLen = (fieldDataCount*2) - asciiAddressLen - SRECORD_CHECKSUM_LEN;
	/* Bailout if we get an invalid data length */
	if (asciiDataLen < 0 || asciiDataLen > SRECORD_MAX_DATA_LEN)
		return SRECORD_ERROR_INVALID_RECORD;

	/* Size check for final data field and checksum field */
	if (strlen(recordBuff) < (unsigned int)(SRECORD_ADDRESS_OFFSET+asciiAddressLen+asciiDataLen+SRECORD_CHECKSUM_LEN))
		return SRECORD_ERROR_INVALID_RECORD;

	dataOffset = SRECORD_ADDRESS_OFFSET+asciiAddressLen;

	/* Loop through each ASCII hex byte of the data field, pull it out into hexBuff,
	 * convert it and store the result in the data buffer of the S-Record */
	for (i = 0; i < asciiDataLen/2; i++) {
		/* Times two i because every byte is represented by two ASCII hex characters */
		strncpy(hexBuff, recordBuff+dataOffset+2*i, SRECORD_ASCII_HEX_BYTE_LEN);
		hexBuff[SRECORD_ASCII_HEX_BYTE_LEN] = 0;
		srec->data[i] = mystrtol(hexBuff, (char **)0, 16);
	}
	/* Real data len is divided by two because every byte is represented by two ASCII hex characters */
	srec->dataLen = asciiDataLen/2;

	/* Copy out the checksum ASCII hex encoded byte, and convert it back to a usable integer */
	strncpy(hexBuff, recordBuff+dataOffset+asciiDataLen, SRECORD_CHECKSUM_LEN);
	hexBuff[SRECORD_CHECKSUM_LEN] = 0;
	srec->checksum = mystrtol(hexBuff, (char **)0, 16);

	if (srec->checksum != Checksum_SRecord(srec))
		return SRECORD_ERROR_INVALID_RECORD;

	return SRECORD_OK;
}

/* Utility function to print the information stored in an S-Record */
void Convert_SRecord(const SRecord *srec) {
  int i;
	switch(srec->type)
	  {
	  case SRECORD_TYPE_S7:
	  case SRECORD_TYPE_S8:
	  case SRECORD_TYPE_S9:
	    myputs("entry = ");
	    myputhex(srec->address, 8);
	    myputs("\r\n");
	    //	    entry(srec->address);
	    break;
	  case SRECORD_TYPE_S1:
	  case SRECORD_TYPE_S2:
	  case SRECORD_TYPE_S3:
	    for (i = 0; i < srec->dataLen; i++) uart_write_byte(i+(unsigned char *)(srec->address), srec->data[i]);
	    break;
	  }
}

/* Utility function to calculate the checksum of an S-Record */
uint8_t Checksum_SRecord(const SRecord *srec) {
	uint8_t checksum;
	int fieldDataCount, i;

	/* Compute the record count, address and checksum lengths are halved because record count
	 * is the number of bytes left in the record, not the length of the ASCII hex representation */
	fieldDataCount = SRecord_Address_Lengths[srec->type]/2 + srec->dataLen + SRECORD_CHECKSUM_LEN/2;

	/* Add the count, address, and data fields together */
	checksum = fieldDataCount;
	/* Add each byte of the address individually */
	checksum += (uint8_t)(srec->address & 0x000000FF);
	checksum += (uint8_t)((srec->address & 0x0000FF00) >> 8);
	checksum += (uint8_t)((srec->address & 0x00FF0000) >> 16);
	checksum += (uint8_t)((srec->address & 0xFF000000) >> 24);
	for (i = 0; i < srec->dataLen; i++)
		checksum += srec->data[i];

	/* One's complement the checksum */
	checksum = ~checksum;

	return checksum;
}

char ucmd[100];

unsigned sd_transaction_v(int sdcmd, unsigned arg, unsigned setting)
{
  int i;
  unsigned resp[8], iobuf[512];
  myputchar('\r');
  myputchar('\n');
  sd_transaction(sdcmd, arg, setting, resp, iobuf, sizeof(iobuf)/sizeof(*iobuf));
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

int minion_sd_debug(void)
{
  int i, rca, busy;
  SRecord srec;
  myputs("Hello\n");
  do {
    size_t addr, addr2, data, sdcmd, arg, setting;
    const char *nxt;
    myputchar('\n');
    cli_readline_into_buffer("dbg> ", ucmd, 0);
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
	data = uart_read((unsigned *)addr);
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
	    data = uart_read((unsigned *)addr);
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
	uart_write((unsigned *)addr, data);
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
      case 'S':
	switch (Read_SRecord(ucmd, &srec))
	  {
	  case SRECORD_OK: Convert_SRecord(&srec); break;
	  case SRECORD_ERROR_INVALID_RECORD: myputchar('?'); break;
	  }
	break;
      case 'q':
	break;
      default: myputs("\nunknown command");
      }
	
  } while (*ucmd != 'q');
  myputs("\nGoodbye\n");
  return 0;
}
