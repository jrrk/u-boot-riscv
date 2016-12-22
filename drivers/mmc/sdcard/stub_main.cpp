#include <stdlib.h>
#include "sim_main.h"

static unsigned setting, start, cmd, timeout, arg, data_start, blksiz, blkcnt, align, finish_cmd, finish_data, crc_ok, index_ok, transf_cnt, read_buf[512], write_buf[512], response[8];

int main(int argc, char **argv, char **envp)
{
  verilator_main(argc, argv, envp);
  while (!verilator_finish())
    {
      verilator_loop(setting, start, cmd, timeout, arg, data_start, blksiz, blkcnt, align, &finish_cmd, &finish_data, &crc_ok, &index_ok, &transf_cnt, read_buf, write_buf, response);
    }
  verilator_stop();
  exit(0);
}
