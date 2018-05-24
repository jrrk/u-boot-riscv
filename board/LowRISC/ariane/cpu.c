#include <common.h>
#include <div64.h>

DECLARE_GLOBAL_DATA_PTR;

/*
 * Return the timebase clock frequency
 * i.e. how often the timer decrements
 */
static unsigned long long div_clock = 25;

unsigned long get_tbclk (void)
{
	unsigned long long tmp = 25000000;

	do_div(tmp, div_clock);

	return tmp;
}

int board_init(void)
{
  return 0;
}

int dram_init(void)
{
  gd->flags |= GD_FLG_SKIP_RELOC;
  gd->ram_size =PHYS_SDRAM_0_SIZE;
  return 0;
}

void set_working_fdt_addr(ulong addr)
{

}

int fake_jump_to_copy(void)
{
  early_puts("fake_jump_to_copy\n");
  board_init_r(gd, 0);
}
