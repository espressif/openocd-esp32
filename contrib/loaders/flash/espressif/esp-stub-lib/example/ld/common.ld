PHDRS
{
  text_phdr PT_LOAD;
  data_phdr PT_LOAD;
}

ENTRY(stub_main)

SECTIONS {
  .iram_org = ORIGIN(iram);
  .iram_len = ABSOLUTE(. + LENGTH(iram));

  .dram_org = ORIGIN(dram);
  .dram_len = ABSOLUTE(. + LENGTH(dram));

  .text : ALIGN(4) {
    *(.stub_desc)
    *(.literal)
    *(.text .text.*)
  } > iram : text_phdr

  .data : ALIGN(4) {
    _data_start = ABSOLUTE(.);
    *(.data)
    *(.rodata .rodata.*)
    *(.sdata .sdata.*)
    *(.sdata2 .sdata2.*)
    _data_end = ABSOLUTE(.);
  } > dram : data_phdr

  .bss (NOLOAD) : ALIGN(4) {
    _bss_start = ABSOLUTE(.);
    *(.bss)
    *(.sbss .sbss.*)
    *(.sbss2 .sbss2.*)
    _bss_end = ABSOLUTE(.);
  } > dram : data_phdr
}
