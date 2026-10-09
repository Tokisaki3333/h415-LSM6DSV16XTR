ENTRY( _start )

__stack_size = 32768;
PROVIDE( _stack_size = __stack_size );

MEMORY
{
    /* 双核共享 4M Flash 分区:
     *   0x00000000 | V3F 固件        | 64K
     *   0x00010000 | V5F 固件        | 128K   (本核)
     *   0x00030000 | 预留(空出)      | 128K
     *   0x00050000 | 字库区域        | 128K   (已烧录, 常驻 Flash)
     *
     * 绑地址模式: 字库区已烧录常驻, 不参与本链接;
     * 注释掉 FONT 区域与 .font 段, 下载工具不会擦除该区。
     */
    FLASH (rx)      : ORIGIN = 0x00010000, LENGTH = 128K
    /* FONT (rx) : ORIGIN = 0x00050000, LENGTH = 128K  字库区: 已烧录常驻, 注释防下载擦除 */
    RAM_CODE (xrw)  : ORIGIN = 0x200A0000, LENGTH = 128K            /* ITCM */
    RAM_LOAD (xrw)  : ORIGIN = 0x200C0000, LENGTH = 256             /* DTCM 开头 */
    RAM (xrw)       : ORIGIN = 0x200C0100, LENGTH = (256K - 256) + 384K  /* DTCM剩余 + 共享SRAM前端384K */
}

SECTIONS
{
	.init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >FLASH AT>FLASH

    .loadcodelalign : 
    {       
        . = ALIGN(4);
        PROVIDE(_loadcode_lma = .); 
    } >FLASH AT>FLASH 

	.loadcode :
	{
		. = ALIGN(4);
        PROVIDE(_loadcode_vma_start = .);
		*(.load);
		. = ALIGN(4); 
		PROVIDE(_loadcode_vma_end = .);
	}>RAM_LOAD AT>FLASH

    .highcodelalign : 
    {       
        . = ALIGN(4);
        PROVIDE(_highcode_lma = .); 
    } >FLASH AT>FLASH 
    
    .highcode : 
    {
        . = ALIGN(4);
        PROVIDE(_highcode_vma_start = .);
        *(.vector);
		KEEP(*(SORT_NONE(.vector_handler)))
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
        *(.sdata2.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4); 
        PROVIDE(_highcode_vma_end = .);
    } >RAM_CODE AT>FLASH

	.text :
	{
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.handle_reset)))
		. = ALIGN(4);
	} >FLASH AT>FLASH 

    /* 字库段 .font: 绑地址模式无字库数据, 本段注释(防下载擦除字库区)。
     * 内嵌模式才恢复本段与 FONT 区域。 */
    /* .font :
    {
        . = ALIGN(4);
        KEEP(*(.font))
        . = ALIGN(4);
        PROVIDE(_font_end = .);
    } >FONT AT>FONT */

    .highcodelalign1 : 
    {       
        . = ALIGN(4);
        PROVIDE(_highcode_lma1 = .); 
    } >FLASH AT>FLASH 

	.fini :
	{
	    . = ALIGN(4);
        PROVIDE(_highcode_vma_start1 = .);
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >RAM_CODE AT>FLASH

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >RAM_CODE AT>FLASH 
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >RAM_CODE AT>FLASH 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >RAM_CODE AT>FLASH 
	
	.ctors          :
	{
	  /* gcc uses crtbegin.o to find the start of
	     the constructors, so we make sure it is
	     first.  Because this is a wildcard, it
	     doesn't matter if the user does not
	     actually link against crtbegin.o; the
	     linker won't look for a file to match a
	     wildcard.  The wildcard also means that it
	     doesn't matter which directory crtbegin.o
	     is in.  */
	  KEEP (*crtbegin.o(.ctors))
	  KEEP (*crtbegin?.o(.ctors))
	  /* We don't want to include the .ctor section from
	     the crtend.o file until after the sorted ctors.
	     The .ctor section from the crtend file contains the
	     end of ctors marker and it must be last */
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
	  KEEP (*(SORT(.ctors.*)))
	  KEEP (*(.ctors))
	} >RAM_CODE AT>FLASH 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	  . = ALIGN(4); 
      PROVIDE(_highcode_vma_end1 = .);
	} >RAM_CODE AT>FLASH 

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH	

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{
		. = ALIGN(4);
    	*(.gnu.linkonce.r.*)
    	*(.data .data.*)
    	*(.gnu.linkonce.d.*)
		. = ALIGN(8);
    	PROVIDE( __global_pointer$ = . + 0x800 );
    	*(.sdata .sdata.*)
		*(.sdata2.*)
    	*(.gnu.linkonce.s.*)
    	. = ALIGN(8);
    	*(.srodata.cst16)
    	*(.srodata.cst8)
    	*(.srodata.cst4)
    	*(.srodata.cst2)
    	*(.srodata .srodata.*)
    	. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH

	.bss :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
  	    *(.sbss*)
        *(.gnu.linkonce.sb.*)
		*(.bss*)
     	*(.gnu.linkonce.b.*)		
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
	} >RAM AT>FLASH

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
    {
        PROVIDE( _heap_end = . );    
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
    } >RAM 

}



