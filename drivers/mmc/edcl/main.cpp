/**
 * @file
 * @copyright  Copyright 2016 GNSS Sensor Ltd. All right reserved.
 * @author     Sergey Khabarov - sergeykhbr@gmail.com
 * @brief      Test application to verify UDP/EDCL transport library.
 */

#include "api_core.h"
#include "iservice.h"
#include "iudp.h"
#include "ithread.h"
#include "itap.h"
#include "ielfloader.h"
#include "main.h"
#include <stdio.h>
#include <string>
#include <assert.h>

using namespace debugger;

/// Use it if configuration file was not found or failed.
const char *default_config = 
"{"
"  'GlobalSettings':{"
"    'SimEnable':false,"
"    'GUI':'false',"
"    'ScriptFile':''},"
"  'Services':["
"    {"
"          'Class':'EdclServiceClass',"
"          'Instances':["
"            {"
"              'Name':'edcltap',"
"              'Attr':["
"                ['LogLevel',0x1],"
"                ['Transport','udpedcl'],"
"                ['seq_cnt',0x1]]}]},"
"        {"
"          'Class':'ElfLoaderServiceClass',"
"          'Instances':["
"            {"
"              'Name':'loader0',"
"              'Attr':["
"                ['LogLevel',0x4],"
"                ['Tap','edcltap'],"
"                ['VerifyEna',true]]}]},"
"        {"
"          'Class':'UdpServiceClass',"
"          'Instances':["
"            {"
"              'Name':'udpboard',"
"              'Attr':["
"                ['LogLevel',0x1],"
"                ['Timeout',0x190],"
"                ['BlockingMode',true],"
"                ['HostIP','192.168.0.53'],"
"                ['BoardIP','192.168.0.48']]},"
"            {"
"              'Name':'udpedcl',"
"              'Attr':["
"                ['LogLevel',0x1],"
"                ['Timeout',0x3e8],"
"                ['BlockingMode',true],"
"                ['HostIP','192.168.0.53'],"
"                ['BoardIP','192.168.0.51']]}]}]}";

static AttributeType Config;
static ITap *edcl_itap;
static IElfLoader *iloader_;
static char tmp[256], old[256];

int edcl_read(uint64_t addr, int bytes, uint8_t *obuf)
{
  int status;
  if (!edcl_itap) edcl_main();
  status = edcl_itap->read(addr,bytes,obuf);
#ifdef VERBOSE
  sprintf(tmp, "edcl_read(0x%.8lX, %d, 0x%.8lX);", addr, bytes, *(uint64_t *)obuf);
  if (strcmp(tmp,old))
    {
      puts(tmp);
      strcpy(old,tmp);
    }
#endif
  return status;
}

int edcl_write(uint64_t addr, int bytes, uint8_t *ibuf)
{
  if (!edcl_itap) edcl_main();
#ifdef VERBOSE
  sprintf(tmp, "edcl_write(0x%.8lX, %d, 0x%.8lX);", addr, bytes, *(uint64_t *)ibuf);
  if (strcmp(tmp,old))
    {
      puts(tmp);
      strcpy(old,tmp);
    }
#endif
  return edcl_itap->write(addr,bytes,ibuf);
}

static int logf;

int edcl_main(void)
{
  // Select configuration input (default/stored file):
  RISCV_init();
  Config.from_config(default_config);
  
  RISCV_set_configuration(&Config);
  
  edcl_itap = static_cast<ITap *>
    (RISCV_get_service_iface("edcltap", IFACE_TAP));
  iloader_ = static_cast<IElfLoader *>
    (RISCV_get_service_iface("loader0", IFACE_ELFLOADER));
}

int edcl_loadelf(const char *elf)
{
  if (!iloader_)
    edcl_main();
  return iloader_->loadFile(elf);
}

void edcl_close(void)
{
  if (logf) close(logf);
  RISCV_cleanup();
}

void log_edcl(const char *fmt, ...)
       {
           int n;
           int size = 100;     /* Guess we need no more than 100 bytes */
           char *p, *np;
           va_list ap;

           if ((p = (char *)malloc(size)) == NULL)
               return;

           while (1) {

               /* Try to print in the allocated space */

               va_start(ap, fmt);
               n = vsnprintf(p, size, fmt, ap);
               va_end(ap);

               /* Check error code */

               if (n < 0)
		 return;

               /* If that worked, return the string */

               if (n < size)
		 {
		   int actual;
		   if (!logf)
		     logf = open("/var/tmp/edcl.log", O_CREAT|O_TRUNC|O_WRONLY, 0700);
		   actual = write(logf, p, n);
		   assert(actual==n);
		   return;
		 }

               /* Else try again with more space */

               size = n + 1;       /* Precisely what is needed */

               if ((np = (char *)realloc (p, size)) == NULL) {
                   free(p);
                   return;
               } else {
                   p = np;
               }
           }
       }

int log_edcl_read(uint64_t addr, int bytes, uint8_t *obuf)
{
  int i, rslt = edcl_read(addr, bytes, obuf);
  log_edcl("edcl_read(%X,%X) =>", addr, bytes);
  for (i = 0; i < bytes; i++) log_edcl(" %.2X", obuf[i]);
  log_edcl("\n");
}

int log_edcl_write(uint64_t addr, int bytes, uint8_t *ibuf)
{
  int i;
  log_edcl("edcl_write(%X,%X) =>", addr, bytes);
  for (i = 0; i < bytes; i++) log_edcl(" %.2X", ibuf[i]);
  log_edcl("\n");
  return edcl_write(addr, bytes, ibuf);
}
