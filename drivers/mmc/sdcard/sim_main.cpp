#include <unistd.h>
#include <fcntl.h>
#include "Vusimv_top.h"
#include "verilated.h"
#include "sim_main.h"

static Vusimv_top *top;
static int tb;
static unsigned rst, sd_clk, setting_i, start_i, cmd_i, timeout_i, arg_i, sd_data_start_i, sd_blksize_i, sd_blkcnt_i, sd_align_i, sd_data_i, nonzero;

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
  if (sd_data_start_i != top->sd_data_start_i)
    {
      sd_data_start_i = top->sd_data_start_i;
      verilator_printf("sd_data_start_i='H%X;\n", sd_data_start_i);
    }
  if (sd_blksize_i != top->sd_blksize_i)
    {
      sd_blksize_i = top->sd_blksize_i;
      verilator_printf("sd_blksize_i='H%X;\n", sd_blksize_i);
    }
  if (sd_blkcnt_i != top->sd_blkcnt_i)
    {
      sd_blkcnt_i = top->sd_blkcnt_i;
      verilator_printf("sd_blkcnt_i='H%X;\n", sd_blkcnt_i);
    }
  if (sd_align_i != top->sd_align_i)
    {
      sd_align_i = top->sd_align_i;
      verilator_printf("sd_align_i='H%X;\n", sd_align_i);
    }
  if (sd_data_i != top->sd_data_i)
    {
      sd_data_i = top->sd_data_i;
      verilator_printf("sd_data_i='H%X;\n", sd_data_i);
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
		 sd_data_start_i = -1;
		 sd_blkcnt_i = -1;
		 sd_blksize_i = -1;
		 sd_align_i = -1;
		 sd_data_i = -1;
                 top->rst = 1;
                 top->sd_clk = 0;
		 top->setting_i = 0;
		 top->start_i = 0;
		 top->cmd_i = 0;
		 top->arg_i = 0;
		 top->timeout_i = 1;
		 top->sd_data_start_i = 0;
		 top->sd_blksize_i = 0;
		 top->sd_blkcnt_i = 0;
		 top->sd_align_i = 0;
		 top->sd_data_i = 0;
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

void verilator_loop(unsigned setting, unsigned start, unsigned cmd, unsigned timeout, unsigned arg,
		    unsigned data_start, unsigned blksiz, unsigned blkcnt, unsigned align,
		    unsigned *finish_cmd, unsigned *finish_data, unsigned *crc_ok, unsigned *index_ok, unsigned *transf_cnt,
		    unsigned read_buf[], unsigned write_buf[], unsigned response[])
{
 top->setting_i = setting;
 top->start_i = start;
 top->cmd_i = cmd;
 top->timeout_i = timeout;
 top->arg_i = arg;
 top->sd_data_start_i = data_start;
 top->sd_blksize_i = blksiz;
 top->sd_blkcnt_i = blkcnt;
 top->sd_align_i = align;
 if (top->sd_rd_o)
   top->sd_data_i = read_buf[top->transf_cnt_o/8];
 verilator_eval();
 top->sd_clk = 1;
 verilator_eval();
 top->sd_clk = 0;
 verilator_eval();
 *finish_cmd = top->finish_cmd_o;
 *finish_data = top->finish_data_o;
 *crc_ok = top->crc_ok_o;
 *index_ok = top->index_ok_o;
 *transf_cnt = top->transf_cnt_o;
 if (top->transf_cnt_o && top->sd_we_o)
   {
   write_buf[top->transf_cnt_o/8] = top->sd_data_o;
   if (top->sd_data_o)
     {
       nonzero = 1;
     }
   }
 response[0] = top->response0_o;
 response[1] = top->response1_o;
 response[2] = top->response2_o;
 response[3] = top->response3_o;
 response[4] = top->wait_o;
 response[5] = top->status_o;
 response[6] = top->packet0_o;
 response[7] = top->packet1_o;
}
