#include <unistd.h>
#include <fcntl.h>
#include "Vusimv_top.h"
#include "verilated.h"
#include "sim_main.h"

static Vusimv_top *top;
static int tb;
static unsigned rst, sd_clk, setting_i, start_i, cmd_i, xmit_i, arg_i;

void verilator_printf(const char *fmt, ...)
{
  uint i;
  va_list args;
  char printbuffer[256];
  va_start(args, fmt);
  i = vsnprintf(printbuffer, sizeof(printbuffer), fmt, args);
  va_end(args);
  write(tb, printbuffer, i);
}

void verilator_stop(void)
{
  close(tb);
  delete top;
}

int verilator_finish(void)
{
  return Verilated::gotFinish();
}

void verilator_eval(void)
{
  if (sd_clk != top->sd_clk)
    {
      sd_clk = top->sd_clk;
      verilator_printf("#1000 sd_clk='H%X;\n", sd_clk);
    }
  if (rst != top->rst)
    {
      rst = top->rst;
      verilator_printf("rst='H%X;\n", rst);
    }
  if (setting_i != top->setting_i)
    {
      setting_i = top->setting_i;
      verilator_printf("setting_i='H%X;\n", setting_i);
    }
  if (start_i != top->start_i)
    {
      start_i = top->start_i;
      verilator_printf("start_i='H%X;\n", start_i);
    }
  if (cmd_i != top->cmd_i)
    {
      cmd_i = top->cmd_i;
      verilator_printf("cmd_i='H%X;\n", cmd_i);
    }
  if (timeout_i != top->timeout_i)
    {
      timeout_i = top->timeout_i;
      verilator_printf("timeout_i='H%X;\n", timeout_i);
    }
  if (arg_i != top->arg_i)
    {
      arg_i = top->arg_i;
      verilator_printf("arg_i='H%X;\n", arg_i);
    }
  top->eval();
}

void verilator_main(int argc, char **argv, char **env) {
                 Verilated::commandArgs(argc, argv);
                 top = new Vusimv_top;
		 tb = open("usimv_top_tb.v",O_CREAT|O_WRONLY|O_TRUNC, 0700);
		 rst = -1;
		 sd_clk = -1;
		 setting_i = -1;
		 start_i = -1;
		 cmd_i = -1;
		 timeout_i = -1;
		 arg_i = -1;
                 top->rst = 1;
                 top->sd_clk = 0;
		 top->setting_i = 0;
		 top->start_i = 0;
		 top->cmd_i = 0;
		 top->timeout_i = 1;
		 top->arg_i = 0;
                 verilator_eval();
                 top->sd_clk = 1;
                 verilator_eval();
                 top->sd_clk = 0;
                 verilator_eval();
                 top->sd_clk = 1;
                 verilator_eval();
                 top->sd_clk = 0;
                 verilator_eval();
                 top->sd_clk = 1;
                 verilator_eval();
                 top->sd_clk = 0;
                 verilator_eval();
                 top->sd_clk = 1;
                 verilator_eval();
                 top->rst = 0;
                 top->sd_clk = 0;
                 verilator_eval();
                 top->sd_clk = 1;
                 verilator_eval();
                 top->sd_clk = 0;
                 verilator_eval();
                 top->sd_clk = 1;
                 verilator_eval();
}

void verilator_loop(unsigned setting, unsigned start, unsigned cmd, unsigned timeout, unsigned arg, unsigned *finish, unsigned *crc_ok, unsigned *index_ok, unsigned response[])
{
 top->setting_i = setting;
 top->start_i = start;
 top->cmd_i = cmd;
 top->timeout_i = timeout;
 top->arg_i = arg;
 verilator_eval();
 top->sd_clk = 1;
 verilator_eval();
 top->sd_clk = 0;
 verilator_eval();
 verilator_eval();
 *finish = top->finish_o;
 *crc_ok = top->crc_ok_o;
 *index_ok = top->index_ok_o;
 response[0] = top->response0_o;
 response[1] = top->response1_o;
 response[2] = top->response2_o;
 response[3] = top->response3_o;
 response[4] = top->wait_o;
 response[5] = top->status_o;
 response[5] = top->packet0_o;
 response[6] = top->packet1_o;
}
