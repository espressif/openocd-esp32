#
# "main" pseudo-component makefile.
#
# (Uses default behaviour of compiling all source files in directory, adding 'include' to include path.)

CFLAGS += -ggdb -I$(BUILD_DIR_BASE)
COMPONENT_ADD_LDFLAGS = -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive

ifdef CONFIG_ESP32_GCOV_ENABLE
CFLAGS += --coverage
endif


ifneq ($(CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME),"")
GEN_UT_APP_CUSTOM_LD_ROOT := $(call dequote,$(COMPONENT_PATH))
GEN_UT_APP_CUSTOM_LD_PATH := $(call dequote,$(abspath $(GEN_UT_APP_CUSTOM_LD_ROOT)/$(call dequote,$(CONFIG_GEN_UT_APP_CUSTOM_LD_FILENAME))))
LINKER_SCRIPTS := \
	$(GEN_UT_APP_CUSTOM_LD_PATH)	

COMPONENT_ADD_LDFLAGS += -L $(GEN_UT_APP_CUSTOM_LD_ROOT) $(addprefix -T ,$(LINKER_SCRIPTS))

COMPONENT_ADD_LINKER_DEPS := $(LINKER_SCRIPTS)
endif
