#include <config.h>
OUTPUT_ARCH(riscv)
__DYNAMIC  =  0;

MEMORY
{
	vectors	: ORIGIN = 0, LENGTH = 0x2000
	ram	: ORIGIN = 0x80000000,
		  LENGTH = 0x10000000
}

SECTIONS
{
	.vectors :
	{
		*(.vectors)
	} > vectors

	__start = .;
	.text : AT (__start) {
		_stext = .;
		*(.text)
		_etext = .;
		*(.lit)
		*(.shdata)
		_endtext = .;
	}  > ram


	 . = ALIGN(4);
	 .u_boot_list : {
		KEEP(*(SORT(.u_boot_list*)));
	 } > ram

	.rodata : {
		*(.rodata);
		*(.rodata.*)
	} > ram

	.shbss :
	{
		*(.shbss)
	} > ram

	.talias :
	{
	}  > ram

	.data : {
		sdata = .;
		_sdata = .;
		*(.data)
		edata = .;
		_edata = .;
		__init_end = .;
	} > ram

	.bss :
	{
		__bss_start = .;
		*(.bss)
		*(COMMON)
		__bss_end = .;
	} > ram
	__end = .;

	/* No stack specification - done manually */

	.stab  0 (NOLOAD) :
	{
		[ .stab ]
	}

	.stabstr  0 (NOLOAD) :
	{
		[ .stabstr ]
	}
}
