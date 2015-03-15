ENTRY(vectors)

SECTIONS {
	/* jailhouse arm inmates have to start at address zero */
        . = 0x0;
        .boot           : { *(.boot) }

        . = ALIGN(4096);
        . = . + 0x1000;
        stack_top = .;
        bss_start = .;
        .bss            : {
                *(.bss)
                *(COMMON)
        }

        . = ALIGN(4);
        .text           : {
		*(.vectors)
		*(.text)
		*(.text.*)
		*(.gnu.linkonce.t.*)
		*(.plt)
		*(.gnu_warning)
		*(.gcc_execpt_table)
		*(.glue_7)
		*(.glue_7t)
		*(.vfp11_veneer)
		*(.ARM.extab)
		*(.gnu.linkonce.armextab.*)
        }

        .rodata         : {
                *(.rodata)
        }

	.rodata1 : {
	   __rodata1_start = .;
	   *(.rodata1)
	   *(.rodata1.*)
	   __rodata1_end = .;
	}

        .data           : {
		__data_start = .;
		*(.data)
		*(.data.*)
		*(.gnu.linkonce.d.*)
		*(.jcr)
		*(.got)
		*(.got.plt)
		__data_end = .;
        }

	.sdata2 : {
	   __sdata2_start = .;
	   *(.sdata2)
	   *(.sdata2.*)
	   *(.gnu.linkonce.s2.*)
	   __sdata2_end = .;
	}
	
	.sbss2 : {
	   __sbss2_start = .;
	   *(.sbss2)
	   *(.sbss2.*)
	   *(.gnu.linkonce.sb2.*)
	   __sbss2_end = .;
	}
}
