#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

CFLAGS += -ggdb
COMPONENT_ADD_LDFLAGS = -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive

ifdef CONFIG_ESP32_GCOV_ENABLE
CFLAGS += --coverage
endif
