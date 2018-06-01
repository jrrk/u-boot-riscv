#include <common.h>
#include <div64.h>
#include <asm/lowrisc-sdhi.h>

extern int lowrisc_initialize(u8 dev_num, int base_addr);

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
  dcache_disable();
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

int board_eth_init(bd_t *bis)
{
  env_set("ethaddr", NULL);
  return lowrisc_initialize(0, 0x40020000);
}

unsigned long timer_read_counter(void)
{
#ifdef ARIANE
  return read_csr(0xC01) / 10;
#else  
  return read_csr(0xB00) / 10;
#endif  
}

int board_mmc_init(bd_t *bis)
{
        return lowrisc_init(0x40010000, 0, SH_SDHI_QUIRK_64BIT_BUF);
}
