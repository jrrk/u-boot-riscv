#include <stdlib.h>
#include "sim_main.h"

static unsigned setting, start, cmd, xmit, arg, finish, crc_ok, index_ok, response[4];

int main(int argc, char **argv, char **envp)
{
  verilator_main(argc, argv, envp);
  while (!verilator_finish())
    {
      verilator_loop(setting, start, cmd, xmit, arg, &finish, &crc_ok, &index_ok, response);
    }
  verilator_stop();
  exit(0);
}
