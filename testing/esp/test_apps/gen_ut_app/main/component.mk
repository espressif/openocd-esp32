#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

ifdef CONFIG_ESP32_GCOV_ENABLE
CFLAGS += --coverage
endif
